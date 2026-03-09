"""RAMMS TUI — main application."""

from __future__ import annotations

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.reactive import reactive
from textual import work
from textual.widgets import Footer, Header, TabbedContent, TabPane

from ramms_tools.unreal_remote import UnrealRemote
from ramms_tools.tui.widgets import ConnectionBar
from ramms_tools.tui.pages import ArmPage, DashboardPage, IMUPage, MebotPage


class RammsTUI(App):
    """Terminal UI for monitoring and controlling RAMMS robotics systems."""

    TITLE = "RAMMS Control"
    SUB_TITLE = "Robotics Dashboard"

    CSS = """
    Screen {
        layout: vertical;
    }
    TabbedContent {
        height: 1fr;
    }
    TabPane {
        padding: 0;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("r", "refresh", "Refresh"),
        Binding("h", "home_all", "Home All"),
        Binding("1", "tab('dashboard')", "Dashboard", show=False),
        Binding("2", "tab('arm')", "Arm", show=False),
        Binding("3", "tab('mebot')", "Mebot", show=False),
        Binding("4", "tab('imu')", "IMU", show=False),
    ]

    connected: reactive[bool] = reactive(False)
    ue: UnrealRemote | None = None
    arm_comp = None  # RemoteObjectProxy | None
    mebot_comp = None  # RemoteObjectProxy | None
    gripper_comp = None  # RemoteObjectProxy | None

    def __init__(self, host: str = "127.0.0.1", port: int = 30010,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self._host = host
        self._port = port

    def compose(self) -> ComposeResult:
        yield Header()
        yield ConnectionBar(id="conn-bar")
        with TabbedContent(id="tabs"):
            with TabPane("Dashboard", id="tab-dashboard"):
                yield DashboardPage(id="page-dashboard")
            with TabPane("🦾 Arm", id="tab-arm"):
                yield ArmPage(id="page-arm")
            with TabPane("🤖 Mebot", id="tab-mebot"):
                yield MebotPage(id="page-mebot")
            with TabPane("🧭 IMU", id="tab-imu"):
                yield IMUPage(id="page-imu")
        yield Footer()

    def on_mount(self) -> None:
        bar = self.query_one("#conn-bar", ConnectionBar)
        bar.host_label = f"{self._host}:{self._port}"

        self.ue = UnrealRemote(host=self._host, http_port=self._port)

        # Start connection check loop
        self.set_interval(5.0, self._check_connection)
        self._check_connection()

        # Activate dashboard by default
        self._activate_page("tab-dashboard")

    @work(exclusive=True, thread=True)
    def _check_connection(self) -> None:
        if not self.ue:
            return
        was_connected = self.connected
        now_connected = self.ue.ping()

        def _update():
            self.connected = now_connected
            bar = self.query_one("#conn-bar", ConnectionBar)
            bar.connected = now_connected

        self.call_from_thread(_update)

        if now_connected and not was_connected:
            self._discover_components()

    @work(thread=True)
    def _discover_components(self) -> None:
        """Find Kinova and Mebot components in the level."""
        if not self.ue:
            return

        # Kinova arm
        try:
            results = self.ue.find_actors_by_component("KinovaGen3")
            if results:
                self.arm_comp = self.ue.actor(results[0]["component_path"])
        except Exception:
            pass

        # Mebot controller
        try:
            results = self.ue.find_actors_by_component("MebotController")
            if results:
                self.mebot_comp = self.ue.actor(results[0]["component_path"])
        except Exception:
            pass

        # Gripper
        try:
            results = self.ue.find_actors_by_component("GripperController")
            if results:
                self.gripper_comp = self.ue.actor(results[0]["component_path"])
        except Exception:
            pass

    def on_tabbed_content_tab_activated(
        self, event: TabbedContent.TabActivated
    ) -> None:
        self._activate_page(event.pane.id or "")

    def _activate_page(self, pane_id: str) -> None:
        pages = {
            "tab-dashboard": "page-dashboard",
            "tab-arm": "page-arm",
            "tab-mebot": "page-mebot",
            "tab-imu": "page-imu",
        }
        for tab_id, page_id in pages.items():
            try:
                page = self.query_one(f"#{page_id}")
                if hasattr(page, "activate") and hasattr(page, "deactivate"):
                    if tab_id == pane_id:
                        page.activate()
                    else:
                        page.deactivate()
            except Exception:
                pass

    def action_refresh(self) -> None:
        self._check_connection()
        self._discover_components()
        # Force refresh the active page
        for page_id in ("page-dashboard", "page-arm", "page-mebot"):
            try:
                page = self.query_one(f"#{page_id}")
                if hasattr(page, "_active") and page._active:
                    if hasattr(page, "_refresh_dashboard"):
                        page._refresh_dashboard()
                    elif hasattr(page, "_refresh_angles"):
                        page._refresh_angles()
                    elif hasattr(page, "_refresh_motors"):
                        page._refresh_motors()
            except Exception:
                pass

    def action_home_all(self) -> None:
        """Home all joints and motors."""
        try:
            arm = self.query_one("#page-arm", ArmPage)
            if arm:
                arm._home_all()
        except Exception:
            pass
        try:
            mebot = self.query_one("#page-mebot", MebotPage)
            if mebot:
                mebot._home_all()
        except Exception:
            pass

    def action_tab(self, tab_name: str) -> None:
        tabs = self.query_one("#tabs", TabbedContent)
        tabs.active = f"tab-{tab_name}"
