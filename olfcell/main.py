from os.path import join, dirname
from pathlib import Path
import trio
from base_kivy_app.app import BaseKivyApp, run_app_async as run_app_async_base
from base_kivy_app.graphics import HighightButtonBehavior, BufferImage
import logging

from kivy.lang import Builder
from kivy.factory import Factory
from kivy.properties import ObjectProperty, StringProperty, BooleanProperty, \
    NumericProperty
from kivy.uix.boxlayout import BoxLayout

import olfcell
from olfcell.widget import (
    ValveBoardWidget, MFCWidget, RPIPinWidget, VideoPlayer, ExperimentStages
)

__all__ = ('OlfCellApp', 'run_app')


class MainView(BoxLayout):
    """The root widget displayed in the GUI.
    """

    pass


class OlfCellApp(BaseKivyApp):
    """The app which runs the main Ceed GUI.
    """

    _config_props_ = (
        'last_directory', 'n_valve_boards', 'n_mfc'
    )

    _config_children_ = {
        'valve_boards': 'valve_boards', 'mfcs': 'mfcs',
        'rpi_pins': 'rpi_pins', 'stage': 'stage', 'player': 'player',
    }

    last_directory = StringProperty('~')
    """The last directory opened in the GUI.
    """

    kv_loaded = False
    """For tests, we don't want to load kv multiple times so we only load kv if
    it wasn't loaded before.
    """

    yesno_prompt = ObjectProperty(None, allownone=True)
    '''Stores a instance of :class:`YesNoPrompt` that is automatically created
    by this app class. That class is described in ``base_kivy_app/graphics.kv``
    and shows a prompt with yes/no options and callback.
    '''

    _valve_container = None

    n_valve_boards: int = NumericProperty(1)

    valve_boards: list[ValveBoardWidget] = []

    valves_running: bool = BooleanProperty(False)

    _mfc_container = None

    n_mfc: int = NumericProperty(1)

    mfcs: list[MFCWidget] = []

    rpi_pins: RPIPinWidget = None

    player: VideoPlayer = None

    stage: ExperimentStages = ObjectProperty(None)

    central_display: BufferImage = ObjectProperty(None, rebind=True)
    """The :class:`~base_kivy_app.graphics.BufferImage` widget into which the
    camera widget is drawn.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.valve_boards = []
        self.mfcs = []
        self.player = VideoPlayer(app=self)
        self.stage = ExperimentStages(
            app=self,
            stage_directory=Path(__file__).parent / "data" / "experiments",
        )

        self.fbind(
            'n_valve_boards', self._update_num_io, 'valve_boards',
            'n_valve_boards', '_valve_container', ValveBoardWidget,
            self._track_valves_running
        )
        self.fbind(
            'n_mfc', self._update_num_io, 'mfcs', 'n_mfc',
            '_mfc_container', MFCWidget, None,
        )

    def load_app_kv(self):
        """Loads the app's kv files, if not yet loaded.
        """
        if OlfCellApp.kv_loaded:
            return
        OlfCellApp.kv_loaded = True

        base = dirname(__file__)
        Builder.load_file(join(base, 'olfcell_style.kv'))

    def build(self):
        self.load_app_kv()
        self.yesno_prompt = Factory.FlatYesNoPrompt()
        self.player.create_widgets()

        root = MainView()
        return super().build(root)

    def on_start(self):
        self.set_tittle()
        HighightButtonBehavior.init_class()

        self.load_app_settings_from_file()
        self.apply_app_settings()

        self.stage.load_protocols()

        logging.root.setLevel(logging.INFO)

    def apply_config_property(self, name, value):
        setattr(self, name, value)
        if name in {'n_valve_boards', 'n_mfc'}:
            self.property(name).dispatch(self)

    def set_tittle(self, *largs):
        """Periodically called by the Kivy Clock to update the title.
        """
        from kivy.core.window import Window

        Window.set_title(f'OlfCell v{olfcell.__version__}, CPL lab')

    def check_close(self):
        if self.stage.playing:
            self._close_message = 'Cannot close during an experiment.'
            return False
        return True

    def clean_up(self):
        super().clean_up()
        HighightButtonBehavior.uninit_class()
        self.dump_app_settings_to_file()

        for dev in self.valve_boards[:]:
            dev.stop()
        for dev in self.mfcs[:]:
            dev.stop()

        if self.rpi_pins is not None:
            self.rpi_pins.stop()

        self.player.clean_up()
        self.stage.stop()

    def _track_valves_running(self, *args):
        was_running = self.valves_running
        running = True
        for dev in self.valve_boards:
            running = running and dev.is_running
        self.valves_running = running

    def _update_num_io(
            self, widgets_name, n_items_name, widgets_container_name,
            widget_cls, callback, *args):
        n_items = getattr(self, n_items_name)
        widgets = getattr(self, widgets_name)
        widgets_container = getattr(self, widgets_container_name)

        if n_items < len(widgets):
            for dev in widgets[n_items:]:
                dev.stop()

                widgets_container.remove_widget(dev)
                widgets.remove(dev)
                dev.funbind("is_running", callback)
        else:
            for _ in range(n_items - len(widgets)):
                dev = widget_cls()
                widgets_container.add_widget(dev)
                widgets.append(dev)

                if callback is not None:
                    dev.fbind("is_running", callback)


def run_app():
    """The function that starts the GUI and the entry point for
    the main script.
    """
    return trio.run(run_app_async_base, OlfCellApp, 'trio')


if __name__ == '__main__':
    run_app()
