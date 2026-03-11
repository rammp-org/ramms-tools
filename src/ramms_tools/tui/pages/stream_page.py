"""TUI page for monitoring RMSS video/data streams."""

from __future__ import annotations

import time
import threading
from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical
from textual.reactive import reactive
from textual.widgets import Static, DataTable, Label, Button, Input
from textual import work

from ramms_tools.streaming.client import StreamClient, ChannelStats
from ramms_tools.streaming.compression import has_jpeg, has_lz4


class StreamPage(Static):
    """Real-time stream monitoring page.

    Shows connection status, per-channel FPS/bandwidth/latency,
    and allows subscribe/unsubscribe control.
    """

    DEFAULT_CSS = """
    StreamPage {
        layout: vertical;
        height: 1fr;
        padding: 1;
    }
    #stream-header {
        height: 3;
        padding: 0 1;
    }
    #stream-status {
        color: $text-muted;
    }
    #stream-connect-bar {
        height: 3;
        padding: 0 1;
    }
    #stream-host-input {
        width: 30;
    }
    #stream-port-input {
        width: 10;
    }
    .stream-btn {
        margin: 0 1;
        min-width: 12;
    }
    #stream-table {
        height: 1fr;
    }
    #stream-footer {
        height: 3;
        padding: 0 1;
        color: $text-muted;
    }
    """

    _active: reactive[bool] = reactive(False)
    _stream_connected: reactive[bool] = reactive(False)

    def __init__(self, stream_host: str = "127.0.0.1",
                 stream_port: int = 30030, **kwargs) -> None:
        super().__init__(**kwargs)
        self._stream_host = stream_host
        self._stream_port = stream_port
        self._client: StreamClient | None = None
        self._refresh_timer = None

    def compose(self) -> ComposeResult:
        with Vertical(id="stream-header"):
            yield Label("📡 RMSS Stream Monitor", id="stream-title")
            yield Label("Disconnected", id="stream-status")

        with Horizontal(id="stream-connect-bar"):
            yield Input(
                value=self._stream_host,
                placeholder="Host",
                id="stream-host-input",
            )
            yield Input(
                value=str(self._stream_port),
                placeholder="Port",
                id="stream-port-input",
            )
            yield Button("Connect", id="stream-connect-btn", classes="stream-btn")
            yield Button("Disconnect", id="stream-disconnect-btn",
                         classes="stream-btn", disabled=True)
            yield Button("Subscribe All", id="stream-sub-btn",
                         classes="stream-btn", disabled=True)

        yield DataTable(id="stream-table")

        codecs = []
        if has_jpeg():
            codecs.append("JPEG")
        if has_lz4():
            codecs.append("LZ4")
        codec_str = ", ".join(codecs) if codecs else "none (install Pillow / lz4)"
        yield Label(
            f"Decompression: {codec_str}  |  Auto-decompress: ON",
            id="stream-footer",
        )

    def on_mount(self) -> None:
        table = self.query_one("#stream-table", DataTable)
        table.add_columns(
            "Channel", "Type", "Frames", "FPS",
            "Bandwidth", "Compressed", "Dropped", "Last Seq",
        )

    def activate(self) -> None:
        self._active = True
        if self._refresh_timer is None:
            self._refresh_timer = self.set_interval(0.5, self._refresh_stats)

    def deactivate(self) -> None:
        self._active = False
        if self._refresh_timer is not None:
            self._refresh_timer.stop()
            self._refresh_timer = None

    # ── Button handlers ──────────────────────────────────────────────

    def on_button_pressed(self, event: Button.Pressed) -> None:
        btn_id = event.button.id
        if btn_id == "stream-connect-btn":
            self._do_connect()
        elif btn_id == "stream-disconnect-btn":
            self._do_disconnect()
        elif btn_id == "stream-sub-btn":
            self._do_subscribe_all()

    @work(thread=True)
    def _do_connect(self) -> None:
        host = self.query_one("#stream-host-input", Input).value.strip()
        port_str = self.query_one("#stream-port-input", Input).value.strip()
        try:
            port = int(port_str)
        except ValueError:
            port = 30030

        self._stream_host = host
        self._stream_port = port

        try:
            client = StreamClient(host, port, auto_decompress=True)
            client.connect(timeout=3.0)
            client.start()
            self._client = client

            def _update():
                self._stream_connected = True
                self.query_one("#stream-status", Label).update(
                    f"✅ Connected to {host}:{port}"
                )
                self.query_one("#stream-connect-btn", Button).disabled = True
                self.query_one("#stream-disconnect-btn", Button).disabled = False
                self.query_one("#stream-sub-btn", Button).disabled = False

            self.call_from_thread(_update)
        except Exception as exc:

            def _err():
                self.query_one("#stream-status", Label).update(
                    f"❌ Connection failed: {exc}"
                )

            self.call_from_thread(_err)

    def _do_disconnect(self) -> None:
        if self._client:
            self._client.disconnect()
            self._client = None
        self._stream_connected = False
        self.query_one("#stream-status", Label).update("Disconnected")
        self.query_one("#stream-connect-btn", Button).disabled = False
        self.query_one("#stream-disconnect-btn", Button).disabled = True
        self.query_one("#stream-sub-btn", Button).disabled = True

    @work(thread=True)
    def _do_subscribe_all(self) -> None:
        if self._client:
            try:
                self._client.subscribe(channels=None, compression="none")

                def _ok():
                    self.query_one("#stream-status", Label).update(
                        f"✅ Subscribed to all channels"
                    )

                self.call_from_thread(_ok)
            except Exception as exc:

                def _err():
                    self.query_one("#stream-status", Label).update(
                        f"⚠️ Subscribe failed: {exc}"
                    )

                self.call_from_thread(_err)

    # ── Stats refresh ────────────────────────────────────────────────

    def _refresh_stats(self) -> None:
        if not self._active or not self._client:
            return

        table = self.query_one("#stream-table", DataTable)
        stats = self._client.get_channel_stats()

        table.clear()
        for ch_id in sorted(stats.keys()):
            cs = stats[ch_id]
            fps = cs.fps
            avg_size = cs.bytes_total / max(cs.frames, 1)
            avg_comp = cs.bytes_compressed / max(cs.frames, 1)
            bw_mbps = fps * avg_size / (1024 * 1024)
            ratio = (
                f"{avg_comp / avg_size * 100:.0f}%"
                if avg_size > 0 and avg_comp != avg_size
                else "raw"
            )

            table.add_row(
                str(ch_id),
                "—",
                str(cs.frames),
                f"{fps:.1f}",
                f"{bw_mbps:.2f} MB/s",
                ratio,
                str(cs.dropped),
                str(cs.last_seq),
            )

        # Update status with totals
        if self._stream_connected:
            total_fps = sum(s.fps for s in stats.values())
            total_msgs = self._client.messages_received
            uptime = self._client.uptime
            mins, secs = divmod(int(uptime), 60)
            self.query_one("#stream-status", Label).update(
                f"✅ Connected | {total_fps:.1f} fps | "
                f"{total_msgs} msgs | uptime {mins}m{secs:02d}s"
            )
