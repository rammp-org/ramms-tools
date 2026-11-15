"""The TUI's Robot page against a fake world (offline; needs textual)."""
import asyncio

import pytest

pytest.importorskip("textual")

from textual.app import App
from textual.widgets import Button, DataTable, Select, Static, Switch

from fake_remote import robot_world
from test_sensing import FakeSensingBridge
from ramms_tools.sensing import BRIDGE as SENSING_BRIDGE
from ramms_tools.tui.pages import RobotPage


class PageApp(App):
    def __init__(self, ue):
        super().__init__()
        self.ue = ue
        self.connected = True

    def compose(self):
        yield RobotPage(id="page-robot")


async def _until(pilot, cond, timeout=5.0):
    loop = asyncio.get_running_loop()
    end = loop.time() + timeout
    while not cond():
        if loop.time() > end:
            raise AssertionError("timed out waiting for the page")
        await pilot.pause(0.05)


def run(coro):
    return asyncio.run(coro)


def test_robot_page_builds_and_commands():
    ue, path, comps = robot_world("gen2")
    bridge = FakeSensingBridge()
    ue.objects[SENSING_BRIDGE] = bridge
    bridge.AddSensorJson({"ActorPath": path, "MountJson": '{"name": "front_tof", "kind": "ToF"}'})
    surface = comps["ControlSurface"]

    async def scenario():
        app = PageApp(ue)
        async with app.run_test(size=(180, 60)) as pilot:
            page = app.query_one(RobotPage)
            page.activate()
            await _until(pilot, lambda: page.robot is not None and page._buttons)
            assert "Gen2Robot_C_0" in str(app.query_one("#robot-title", Static).render())
            # Mode selector filled from the selector, with the active mode shown.
            await _until(pilot, lambda: app.query_one("#robot-mode", Select).value == "differential")
            # Enum selectors and action buttons per group (curb.view.*, camera.next ...), not the dedicated ones.
            assert "sel-curb_view_sensing" in page._selects and "btn-camera_next" in page._buttons
            assert "btn-curb_negotiate" not in page._buttons
            assert app.query_one("#curb-detect", Button).display and not app.query_one("#curb-style", Select).display
            # Live readback and the curb panel.
            await _until(pilot, lambda: "seat.height" in page._readback_rows)
            await _until(pilot, lambda: "Ready" in str(app.query_one("#curb-status", Static).render()))
            await _until(pilot, lambda: "front_tof" in page._sensor_rows)
            await _until(pilot, lambda: "nearest 41.2 cm" in str(app.query_one("#robot-sensors", DataTable).get_cell("front_tof", "reading")))
            # A button triggers its control; the assist switch sets curb.assist.
            app.query_one("#btn-camera_next", Button).press()
            await _until(pilot, lambda: "camera.next" in surface.triggered)
            app.query_one("#curb-assist", Switch).value = True
            await _until(pilot, lambda: surface.values["curb.assist"] == 1.0)
            # Choosing a drive mode switches it.
            app.query_one("#robot-mode", Select).value = "holonomic"
            await _until(pilot, lambda: comps["DriveModeSelector"].active == "holonomic")
    run(scenario())


def test_robot_page_gen1_style_and_no_sensing():
    ue, path, comps = robot_world("gen1")          # no sensing bridge in this world

    async def scenario():
        app = PageApp(ue)
        async with app.run_test(size=(180, 60)) as pilot:
            page = app.query_one(RobotPage)
            page.activate()
            await _until(pilot, lambda: page.robot is not None and page._buttons)
            style = app.query_one("#curb-style", Select)
            assert style.display and not app.query_one("#curb-detect", Button).display
            await _until(pilot, lambda: "not available" in str(app.query_one("#robot-sensor-status", Static).render()))
            style.value = 1
            await _until(pilot, lambda: comps["ControlSurface"].values["curb.style"] == 1.0)
    run(scenario())


def test_robot_page_refills_its_selectors_after_a_rebuild():
    """A drive-mode change (or Refresh) rebuilds the control selectors blank: the next polls must fill them again,
    and an unreadable value (NaN) must not stop the poll."""
    ue, path, comps = robot_world("gen2")

    async def scenario():
        app = PageApp(ue)
        async with app.run_test(size=(180, 60)) as pilot:
            page = app.query_one(RobotPage)
            page.activate()
            await _until(pilot, lambda: page.robot is not None and page._buttons)
            sel = lambda: app.query_one("#sel-curb_view_labels", Select)        # noqa: E731
            await _until(pilot, lambda: sel().value == 1)
            app.query_one("#robot-mode", Select).value = "holonomic"           # rebuilds the controls
            await _until(pilot, lambda: comps["DriveModeSelector"].active == "holonomic")
            await _until(pilot, lambda: sel().value == 1, timeout=10.0)
            comps["ControlSurface"].values["curb.view.sensing"] = float("nan")  # an unreadable answer
            await _until(pilot, lambda: page._tick > 6, timeout=10.0)
            assert "poll:" not in str(app.query_one("#robot-status-bar", Static).render())
    run(scenario())


def test_robot_page_polls_one_at_a_time_on_a_slow_engine():
    """A thread worker cannot be cancelled: with the engine slower than the poll interval, pollers must not pile up."""
    import threading
    import time

    ue, path, comps = robot_world("gen1")
    fake = comps["ControlSurface"]
    state = {"inflight": 0, "max": 0}
    lock = threading.Lock()

    def slow(func, p):
        if func in ("GetControlValue", "GetControlText"):
            with lock:
                state["inflight"] += 1
                state["max"] = max(state["max"], state["inflight"])
            time.sleep(0.15)
            with lock:
                state["inflight"] -= 1

    async def scenario():
        app = PageApp(ue)
        async with app.run_test(size=(180, 60)) as pilot:
            page = app.query_one(RobotPage)
            page.activate()
            await _until(pilot, lambda: page.robot is not None and page._buttons)
            fake.on_call = slow
            await pilot.pause(2.0)
            fake.on_call = None
            assert page.polls_skipped > 0
            # A poll of a robot no longer shown is dropped.
            before = str(app.query_one("#curb-phase", Static).render())
            page._show_state({"gen": page._robot_gen - 1, "phase": "Stale"})
            assert str(app.query_one("#curb-phase", Static).render()) == before
    run(scenario())
    assert state["max"] == 1
