import csv
from pathlib import Path
from threading import Thread
import trio
from math import ceil
import pathlib
from time import perf_counter, sleep
import os.path
from typing import List, Dict, Optional, Tuple
from kivy_trio.to_trio import kivy_run_in_async, mark, KivyEventCancelled
from kivy_trio.to_kivy import AsyncKivyEventQueue
from pymoa_remote.threading import ThreadExecutor
from pymoa_remote.socket.websocket_client import WebSocketExecutor
from pymoa_remote.client import Executor
from base_kivy_app.app import app_error
from base_kivy_app.utils import pretty_time, yaml_dumps
from tree_config import read_config_from_object

from kivy.properties import ObjectProperty, StringProperty, BooleanProperty, \
    NumericProperty, ListProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.core.audio import SoundLoader
from kivy.graphics.texture import Texture
from kivy.factory import Factory
from kivy.uix.widget import Widget
from kivy.event import EventDispatcher

from olfcell.device import (
    MODIOBase, MODIOBoard, VirtualMODIOBoard, MFCBase, MFC, VirtualMFC
)

__all__ = ('ValveBoardWidget', 'MFCWidget', 'ExperimentStages')


ProtocolItem = tuple[float, list[bool | None], list[float | None]]
FlatProtocolItem = tuple[
    float, list[tuple['ValveBoardWidget', dict[str, bool]]],
    list[tuple['MFCWidget', float]]
]


class ExecuteDevice:

    __events__ = ('on_data_update', )

    _config_props_ = (
        'virtual', 'remote_server', 'remote_port', 'unique_dev_id',
    )

    remote_server: str = StringProperty('')

    remote_port: int = NumericProperty(0)

    device = None

    virtual = BooleanProperty(False)

    unique_dev_id: str = StringProperty('')

    is_running: bool = BooleanProperty(False)

    def on_data_update(self, *args):
        pass

    async def _run_device(self, executor):
        raise NotImplementedError

    async def run_device(self):
        if self.remote_server and not self.virtual:
            async with trio.open_nursery() as nursery:
                async with WebSocketExecutor(
                        nursery=nursery, server=self.remote_server,
                        port=self.remote_port) as executor:
                    async with executor.remote_instance(
                            self.device, self.unique_dev_id):
                        async with self.device:
                            await self._run_device(executor)
        else:
            async with ThreadExecutor() as executor:
                async with executor.remote_instance(
                        self.device, self.unique_dev_id):
                    async with self.device:
                        await self._run_device(executor)

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError


class ValveBoardWidget(BoxLayout, ExecuteDevice):

    _config_props_ = ('dev_address', 'com_port', 'reverse_relay')

    dev_address: int = NumericProperty(0)

    com_port: str = StringProperty('')

    reverse_relay: bool = BooleanProperty(False)

    device: Optional[MODIOBase] = ObjectProperty(
        None, allownone=True, rebind=True)

    _event_queue: Optional[AsyncKivyEventQueue] = None

    async def _run_device(self, executor: Executor):
        device = self.device
        async with self._event_queue as queue:
            async for low, high, kwargs in queue:
                await device.write_states(high, low, **kwargs)

    @app_error
    @kivy_run_in_async
    def start(self):
        if self.virtual:
            cls = VirtualMODIOBoard
        else:
            cls = MODIOBoard

        self.is_running = True
        self.device = cls(dev_address=self.dev_address, com_port=self.com_port)
        self.device.fbind('on_data_update', self.dispatch, 'on_data_update')
        self._event_queue = AsyncKivyEventQueue()

        try:
            yield mark(self.run_device)
        except KivyEventCancelled:
            pass
        finally:
            self._event_queue.stop()
            self._event_queue = None
            self.device = None
            self.is_running = False

    @app_error
    def stop(self):
        if self._event_queue is not None:
            self._event_queue.stop()

    def set_valves(self, low=(), high=(), **kwargs: bool):
        self._event_queue.add_item(low, high, kwargs)


class MFCWidget(BoxLayout, ExecuteDevice):

    _config_props_ = ('dev_address', 'com_port')

    dev_address: int = NumericProperty(0)

    com_port: str = StringProperty('')

    device: Optional[MFCBase] = ObjectProperty(
        None, allownone=True, rebind=True)

    _event_queue: List[float] = []

    _done = False

    async def _run_device(self, executor: Executor):
        device = self.device
        queue = self._event_queue

        while not self._done:
            i = len(queue)
            if i:
                await device.write_state(queue[i - 1])
                del queue[:i]
            await device.read_state()

    @app_error
    @kivy_run_in_async
    def start(self):
        if self.virtual:
            cls = VirtualMFC
        else:
            cls = MFC

        self.is_running = True
        self.device = cls(dev_address=self.dev_address, com_port=self.com_port)
        self.device.fbind('on_data_update', self.dispatch, 'on_data_update')
        self._event_queue = []
        self._done = False

        try:
            yield mark(self.run_device)
        except KivyEventCancelled:
            pass
        finally:
            self._done = True
            self._event_queue = []
            self.device = None
            self.is_running = False

    @app_error
    def stop(self):
        self._done = True

    def set_value(self, value):
        self._event_queue.append(value)


class ExperimentStages(EventDispatcher):

    playing: bool = BooleanProperty(False)

    protocols_name: list[str] = ListProperty()

    remaining_time: str = StringProperty('00:00:00.0')

    stage_remaining_time: str = StringProperty('00:00:00.0')

    stage_directory: Path = None

    _timer_ts: float = 0

    _total_time: float = 0

    _total_stage_time: float = 0

    _clock_event = None

    _protocol_clock_event = None

    _app = None

    _log_file = None

    protocols: dict[str, list[ProtocolItem]] = {}

    n_stages: int = NumericProperty(0)

    stage_i: int = NumericProperty(0)

    _col_names: tuple[list[str], list[str]] = ([], [])

    def __init__(self, app, stage_directory: Path, **kwargs):
        super().__init__(**kwargs)
        self._app = app
        self.stage_directory = stage_directory
        self.protocols = {}

    def _run_protocol(
            self, protocol: List[FlatProtocolItem]):
        self._total_time = sum(item[0] for item in protocol)
        self._total_stage_time = 0
        rem_time = 0.
        i = -1
        self.n_stages = n = len(protocol)
        ts = 0
        self.stage_i = 0

        def protocol_callback(*args):
            nonlocal i, rem_time, ts, n

            t = perf_counter()
            if t - ts < rem_time:
                return

            i += 1
            if i == n:
                self.stop()
                self._update_clock()
                return
            # update only if not stopping
            self.stage_i = i + 1

            dur, valves, mfcs = protocol[i]

            rem_time += dur
            for board, values in valves:
                board.set_valves(**values)
            for board, value in mfcs:
                board.set_value(value)

            self._total_stage_time = rem_time

        self._protocol_clock_event = Clock.schedule_interval(
            protocol_callback, 0)
        self._clock_event = Clock.schedule_interval(self._update_clock, .25)

        self._timer_ts = ts = perf_counter()
        # this can stop above events
        protocol_callback()

        self._update_clock()

    def _flatten_protocol(
            self, protocol: List[ProtocolItem]) -> List[FlatProtocolItem]:
        valves: List[ValveBoardWidget] = self._app.valve_boards
        mfcs: List[MFCWidget] = self._app.mfcs

        stages = []
        for dur, valve_states, mfc_vals in protocol:
            valve_groups = [
                valve_states[i * 4: (i + 1) * 4]
                for i in range(int(ceil(len(valve_states) / 4)))
            ]
            if len(valve_groups) != len(valves):
                raise ValueError(
                    'The number of valve columns is not the same as valves')
            if len(mfc_vals) != len(mfcs):
                raise ValueError(
                    'The number of MFC columns is not the same as MFCs')

            prepped_valves = []
            for valve_board, states in zip(valves, valve_groups):
                relays = {
                    f'relay_{i}': val for i, val in enumerate(states)
                    if val is not None
                }
                if relays:
                    prepped_valves.append((valve_board, relays))

            prepped_mfcs = [
                (mfc, val) for mfc, val in zip(mfcs, mfc_vals)
                if val is not None
            ]
            stages.append((dur, prepped_valves, prepped_mfcs))

        return stages

    @app_error
    def start(self, key: str):
        from olfcell.main import OlfCellApp
        self.playing = True
        app: OlfCellApp = self._app

        try:
            dev: ExecuteDevice
            for dev in app.valve_boards + app.mfcs:
                if not dev.is_running:
                    raise TypeError('Not all valves/MFCs have been started')

            if key not in self.protocols:
                raise ValueError('Protocol not available')
            protocol = self._flatten_protocol(self.protocols[key])
            if not len(protocol):
                raise ValueError('No protocol available')
        except Exception:
            self.stop()
            raise

        self._run_protocol(protocol)

    def stop(self):
        if self._clock_event is not None:
            self._clock_event.cancel()
            self._clock_event = None

        if self._protocol_clock_event is not None:
            self._protocol_clock_event.cancel()
            self._protocol_clock_event = None

        self.playing = False

    def load_protocols(self) -> None:
        """Called by the GUI when user browses for a file.
        """
        protocols = {}
        for filename in self.stage_directory.glob("*.csv"):
            protocol, header = self._load_protocol(filename)

            name = filename.name[:-4]
            protocols[name] = protocol
            self._col_names = header

        self.protocols = protocols
        self.protocols_name = list(sorted(self.protocols))

    def _load_protocol(
            self, filename: Path
    ) -> tuple[list[ProtocolItem], tuple[list[str], list[str]]]:
        data = filename.read_text(encoding='utf-8-sig')

        reader = csv.reader(data.splitlines())
        header = next(reader)
        if len(header) < 1:
            raise ValueError(
                'The csv file must have at least a duration column')

        if header[0].lower() != 'duration':
            raise ValueError('First column must be named duration')

        i = valve_s = 1
        while i < len(header) and header[i].lower().startswith('valve_'):
            i += 1
        mfc_s = valve_e = i
        while i < len(header) and header[i].lower().startswith('mfc_'):
            i += 1
        mfc_e = i
        if i != len(header):
            raise ValueError(
                f'Reached column "{header[i]}" that does not start with valve_ '
                f'or mfc_')

        valve_names = [header[k][6:] for k in range(valve_s, valve_e)]
        mfc_names = [header[k][4:] for k in range(mfc_s, mfc_e)]

        protocol = []
        for row in reader:
            dur = float(row[0])
            valves = [
                bool(int(row[k])) if row[k] else None
                for k in range(valve_s, valve_e)
            ]
            mfcs = [
                float(row[k]) if row[k] else None for k in range(mfc_s, mfc_e)]

            protocol.append((dur, valves, mfcs))

        return protocol, (valve_names, mfc_names)

    def _update_clock(self, *args):
        ts = self._timer_ts
        if not ts:
            self.remaining_time = self.stage_remaining_time = '00:00:00.0'
            return

        elapsed = perf_counter() - ts
        self.remaining_time = pretty_time(
            max(self._total_time - elapsed, 0), pad=True)
        self.stage_remaining_time = pretty_time(
            max(self._total_stage_time - elapsed, 0), pad=True)
