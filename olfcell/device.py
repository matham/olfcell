from typing import Optional, List, Iterable, Dict
from random import random, randint
from time import perf_counter_ns, sleep
import re
from threading import Lock
import serial
from gpiozero import OutputDevice
from serial.rs485 import RS485Settings

from kivy.properties import ObjectProperty

from pymoa.device import Device
from pymoa.device.digital import DigitalPort
from pymoa.device.analog import AnalogChannel
from pymoa_remote.client import apply_executor, apply_generator_executor


class DeviceContext:

    local_time: float = 0

    async def __aenter__(self):
        await self.open_device()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close_device()

    def open_device(self):
        raise NotImplementedError

    def close_device(self):
        raise NotImplementedError

    @staticmethod
    def get_time():
        return perf_counter_ns() / 1e9


class SerialDevice:
    """Can be shared among multiple :class:`MODIOBoard`.
    """

    devices: Dict[str, 'SerialDevice'] = {}

    _device_count: Dict[str, int] = {}

    port: Optional[serial.Serial] = None

    com_port: str = ''

    _creation_lock = Lock()

    _access_lock: Lock = None

    def __init__(self, com_port: str):
        self.com_port = com_port
        self._access_lock = Lock()

    @classmethod
    def get_open_device(cls, com_port: str) -> 'SerialDevice':
        with cls._creation_lock:
            if com_port in cls.devices:
                cls._device_count[com_port] += 1
                return cls.devices[com_port]

            cls._device_count[com_port] = 1
            dev = cls.devices[com_port] = SerialDevice(com_port=com_port)
            dev._open()
            return dev

    def return_device(self) -> None:
        cls = self.__class__
        com_port = self.com_port
        with cls._creation_lock:
            if com_port not in cls.devices:
                return
            cls._device_count[com_port] -= 1
            if cls._device_count[com_port]:
                return

            dev = cls.devices[com_port]
            del cls._device_count[com_port]
            del cls.devices[com_port]
            dev._close()

    def _open(self):
        ser = serial.Serial()
        ser.port = self.com_port
        ser.baudrate = 19200
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 1
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.writeTimeout = 1
        ser.open()

        self.port = ser

    def _close(self):
        if self.port is not None:
            self.port.close()
            self.port = None

    def write(self, data):
        with self._access_lock:
            self.port.write(data)

    def read(self, n):
        with self._access_lock:
            return self.port.read(n)


class MODIOBase(DigitalPort, DeviceContext):

    _config_props_ = ('dev_address', 'com_port', 'reverse_relay')

    dev_address: int = 0

    com_port: str = ''

    reverse_relay: bool = False

    relay_0: bool = False

    relay_1: bool = False

    relay_2: bool = False

    relay_3: bool = False

    opto_0: bool = False

    opto_1: bool = False

    opto_2: bool = False

    opto_3: bool = False

    analog_0: float = 0

    analog_1: float = 0

    analog_2: float = 0

    analog_3: float = 0

    channel_names: List[str] = [
        f'relay_{i}' for i in range(4)] + [
        f'opto_{i}' for i in range(4)] + [
        f'analog_{i}' for i in range(4)]

    _relay_map: Dict[str, int] = {f'relay_{i}': i for i in range(4)}

    _relay_map_rev: Dict[str, int] = {f'relay_{i}': 3 - i for i in range(4)}

    _analog_map: Dict[str, int] = {f'analog_{i}': i for i in range(4)}

    def __init__(
            self, com_port: str = '', dev_address: int = 0, reverse_relay=False,
            **kwargs):
        self.dev_address = dev_address
        self.com_port = com_port
        self.reverse_relay = reverse_relay
        super().__init__(**kwargs)

    def _combine_write_args(
            self, high: Iterable[str], low: Iterable[str],
            kwargs: Dict[str, bool]):
        relay_map = \
            self._relay_map_rev if self.reverse_relay else self._relay_map
        value = 0

        for item in high:
            kwargs[item] = True
        for item in low:
            kwargs[item] = False
        for item in {
                'relay_0', 'relay_1', 'relay_2', 'relay_3'} - kwargs.keys():
            kwargs[item] = getattr(self, item)

        for name, val in kwargs.items():
            if val:
                value |= 1 << relay_map[name]

        return value

    def update_write_data(self, result):
        self.local_time = self.get_time()
        value, t = result

        self.timestamp = t
        for i in range(4):
            setattr(self, f'relay_{i}', bool((1 << i) & value))

        self.dispatch('on_data_update', self)

    def update_read_data(self, result):
        opto_val, analog_vals, t = result

        self.timestamp = t
        if opto_val is not None:
            for i in range(4):
                setattr(self, f'opto_{i}', bool((1 << i) & opto_val))
        for name, value in analog_vals.items():
            setattr(self, name, value)

        self.dispatch('on_data_update', self)

    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        raise NotImplementedError

    def _read_state(self, opto=True, analog_channels: Iterable[str] = ()):
        raise NotImplementedError

    @apply_executor(callback='update_read_data')
    def read_state(self, opto=True, analog_channels: Iterable[str] = ()):
        return self._read_state(opto, analog_channels)

    @apply_generator_executor(callback='update_read_data')
    def pump_state(self, opto=True, analog_channels: Iterable[str] = ()):
        while True:
            yield self._read_state(opto, analog_channels)


class MODIOBoard(MODIOBase):

    device: Optional[SerialDevice] = None

    @apply_executor
    def open_device(self):
        self.device = SerialDevice.get_open_device(self.com_port)

    @apply_executor
    def close_device(self):
        if self.device is not None:
            self.device.return_device()
            self.device = None

    @apply_executor(callback='update_write_data')
    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        value = self._combine_write_args(high, low, kwargs)
        self.device.write(bytes([self.dev_address | 0b10000000, 0x10, value]))
        return value, self.get_time()

    def _read_state(self, opto=True, analog_channels: Iterable[str] = ()):
        if not opto and not analog_channels:
            raise ValueError('No channels specified to read')

        opto_val = None
        if opto:
            self.device.write(bytes([self.dev_address | 0b10000000, 0x20]))
            opto_val = self.device.read(1)

        analog_vals = {}
        a_map = self._analog_map
        for chan in analog_channels:
            self.device.write(bytes(
                [self.dev_address | 0b10000000, 0x30 | (1 << a_map[chan])]))
            msg = self.device.read(2)
            data = list(msg)
            assert len(data) == 2

            low, high = data
            val = 0
            for i in range(8):
                if (1 << (7 - i)) & low:
                    val |= 1 << i
            if high & 0x02:
                val |= 1 << 8
            if high & 0x01:
                val |= 1 << 9

            analog_vals[chan] = val * 3.3 / 1024

        return opto_val, analog_vals, self.get_time()


class VirtualMODIOBoard(MODIOBase):

    @apply_executor
    def open_device(self):
        pass

    @apply_executor
    def close_device(self):
        pass

    @apply_executor(callback='update_write_data')
    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        sleep(.05)
        value = self._combine_write_args(high, low, kwargs)
        return value, self.get_time()

    def _read_state(self, opto=True, analog_channels: Iterable[str] = ()):
        if not opto and not analog_channels:
            raise ValueError('No channels specified to read')

        sleep(.05)

        opto_val = None
        if opto:
            opto_val = randint(0, 0b1111)

        analog_vals = {}
        for chan in analog_channels:
            analog_vals[chan] = random() * 3.3

        return opto_val, analog_vals, self.get_time()


class MFCBase(AnalogChannel, DeviceContext):

    _config_props_ = ('dev_address', 'com_port')

    dev_address: int = 0

    com_port: str = ''

    state: float = 0

    def __init__(self, dev_address: int = 0, com_port: str = '', **kwargs):
        self.dev_address = dev_address
        self.com_port = com_port
        super().__init__(**kwargs)

    def update_data(self, result):
        self.local_time = self.get_time()
        self.state, self.timestamp = result
        self.dispatch('on_data_update', self)

    async def write_state(self, value: float, **kwargs):
        raise NotImplementedError

    def _read_state(self):
        raise NotImplementedError

    @apply_executor(callback='update_data')
    def read_state(self):
        return self._read_state()

    @apply_generator_executor(callback='update_data')
    def pump_state(self):
        while True:
            yield self._read_state()


class MFC(MFCBase):

    device: Optional[serial.Serial] = None

    _rate_read_pat = None

    _rate_write_pat = None

    @apply_executor
    def open_device(self):
        dev = self.dev_address
        self._rate_read_pat = re.compile(
            rf'!{dev:02X},([0-9.]+)\r\n'.encode('ascii'))
        self._rate_write_pat = re.compile(
            rf'!{dev:02X},S([0-9.]+)\r\n'.encode('ascii'))

        ser = self.device = serial.Serial()
        ser.port = self.com_port
        ser.baudrate = 9600
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 1
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.writeTimeout = 1
        ser.rs485_mode = RS485Settings()
        # for some reason, if we don't open, close, and reopen, every second
        # command times out
        ser.open()
        ser.close()
        ser.open()

        ser.write(f'!{dev:02X},M,D\r'.encode('ascii'))
        read = f'!{dev:02X},MD\r\n'.encode('ascii')
        data = ser.read_until(b'\n')
        if data != read:
            raise IOError(
                f'Failed setting MFC to digital mode. '
                f'Expected "{read}", got "{data}"')

        ser.write(f'!{dev:02X},U,SLPM\r'.encode('ascii'))
        read = f'!{dev:02X},USLPM\r\n'.encode('ascii')
        data = ser.read_until(b'\n')
        if data != read:
            raise IOError(
                f'Failed setting MFC to use SLPM units. '
                f'Expected "{read}", got "{data}"')

    @apply_executor
    def close_device(self):
        if self.device is not None:
            self.device.close()
            self.device = None

    @apply_executor
    def write_state(self, value: float, **kwargs):
        dev = self.dev_address
        ser = self.device

        ser.write(f'!{dev:02X},S,{value:.3f}\r'.encode('ascii'))
        read = f'!{dev:02X},S{value:.3f}\r\n'.encode('ascii')
        data = ser.read_until(b'\n')
        m = re.match(self._rate_write_pat, data)
        if m is None:
            raise IOError(
                f'Failed setting MFC rate. Expected "{read}", got "{data}"')

    def _read_state(self):
        dev = self.dev_address
        ser = self.device

        ser.write(f'!{dev:02X},F\r'.encode('ascii'))
        data = ser.read_until(b'\n')
        m = re.match(self._rate_read_pat, data)
        if m is None:
            raise IOError(f'Failed to read MFC rate. Got "{data}"')
        return float(m.group(1).decode('ascii')), self.get_time()


class VirtualMFC(MFCBase):

    @apply_executor
    def open_device(self):
        pass

    @apply_executor
    def close_device(self):
        pass

    @apply_executor
    def write_state(self, value: float, **kwargs):
        self.state = value

    def _read_state(self):
        sleep(.1)
        state = max(self.state + random() * .01 - .005, 0)
        return state, self.get_time()


class RPIPinOutBase(DigitalPort, DeviceContext):

    _config_props_ = ('tone_pin', 'current_pin')

    tone_pin: int = 0

    current_pin: int = 0

    tone: bool = False

    current: bool = False

    channel_names: List[str] = [
        "tone", "current"
    ]

    def __init__(
            self, tone_pin: int = 0, current_pin: int = 0, **kwargs
    ):
        self.tone_pin = tone_pin
        self.current_pin = current_pin
        super().__init__(**kwargs)

    def update_write_data(self, result):
        self.local_time = self.get_time()
        (high, low), t = result

        self.timestamp = t
        for name in high:
            setattr(self, name, True)
        for name in low:
            setattr(self, name, False)

        self.dispatch('on_data_update', self)

    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        raise NotImplementedError


class RPIPinOut(RPIPinOutBase):

    tone_device: OutputDevice | None = None

    current_device: OutputDevice | None = None

    @apply_executor
    def open_device(self):
        self.tone_device = OutputDevice(
            pin=self.tone_pin, active_high=True, initial_value=False
        )
        self.current_device = OutputDevice(
            pin=self.current_pin, active_high=True, initial_value=False
        )

    @apply_executor
    def close_device(self):
        if self.tone_device is not None:
            self.tone_device.close()
            self.tone_device = None
        if self.current_device is not None:
            self.current_device.close()
            self.current_device = None

    @apply_executor(callback='update_write_data')
    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        device: OutputDevice
        for name in high:
            device = getattr(self, f"{name}_device")
            device.on()
        for name in low:
            device = getattr(self, f"{name}_device")
            device.off()

        return (high, low), self.get_time()


class VirtualRPIPinOut(RPIPinOutBase):

    @apply_executor
    def open_device(self):
        pass

    @apply_executor
    def close_device(self):
        pass

    @apply_executor(callback='update_write_data')
    def write_states(
            self, high: Iterable[str] = (), low: Iterable[str] = (),
            **kwargs: bool):
        sleep(.05)
        return (high, low), self.get_time()
