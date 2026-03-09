"""Connection status bar widget."""

from __future__ import annotations

from textual.reactive import reactive
from textual.widgets import Static


class ConnectionBar(Static):
    """Displays UE connection status at the top of the app."""

    DEFAULT_CSS = """
    ConnectionBar {
        height: 1;
        dock: top;
        padding: 0 1;
        background: $error;
        color: $text;
        text-style: bold;
    }
    ConnectionBar.connected {
        background: $success;
    }
    """

    connected: reactive[bool] = reactive(False)
    host_label: reactive[str] = reactive("")

    def render(self) -> str:
        icon = "●" if self.connected else "○"
        status = "Connected" if self.connected else "Disconnected"
        suffix = f" — {self.host_label}" if self.host_label else ""
        return f" {icon} {status}{suffix}"

    def watch_connected(self, connected: bool) -> None:
        self.set_class(connected, "connected")
