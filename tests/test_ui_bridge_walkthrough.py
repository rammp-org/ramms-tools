"""The walkthrough moves on from the screen: a panel button, its Next button, or a box/POI tap.

ScreenClient stands in for the running UI: it keeps the panel and objects last pushed, and the
events posted since. The operator in these tests reads each step's prompt the way a person would
and taps only what is actually on screen -- never what the prompt lists -- so a step that asks
for something the screen does not show waits forever and fails the test.
"""

from __future__ import annotations

import argparse
import builtins
import itertools

import pytest

import ramms_tools.scripts.ui_bridge_test as ubt
from ramms_tools.scripts.ui_bridge_test import NEXT_BUTTON, StepTaps


class ScreenClient:
    POLL_LIMIT = 200  # polls in one wait with nothing moving it on: the step is stuck

    def __init__(self):
        self.panel = None
        self.objects = []
        self.events = []
        self.seq = 0
        self.idle_polls = 0
        self.polls = 0
        self.scheduled = []  # (poll count to post at, event): a person taking their time
        self.last_tapped = None  # the POI layer dims every POI but this one

    # what the walkthrough pushes
    def update_panel(self, pan):
        self.panel = pan

    def update_detected_objects(self, payload):
        self.objects = list(payload["objects"])  # each push is the full set

    def get_setting(self, name):
        return ""

    def open_camera_feed(self):
        pass

    def close_camera_feed(self):
        pass

    # what comes back
    def post(self, event):
        self.seq += 1
        self.events.append(dict(event, seq=self.seq))
        self.idle_polls = 0
        if event.get("event_type") == "object_tapped" and event.get("instance_id"):
            self.last_tapped = event["instance_id"]

    # what the HUD readback sees (tests with a HUD stub read_pois / read_poi_layer with these)
    def pois(self, root, ks=None):
        rows = []
        for k, o in enumerate(self.objects):
            if ks is not None and k not in ks:
                continue
            dim = self.last_tapped is not None and o.get("instance_id") != self.last_tapped
            rows.append({"k": k, "instance_id": o.get("instance_id", ""), "class": o.get("perception_class", ""),
                         "alpha": 0.4 if dim else 1.0})
        return rows

    def later(self, event, polls):
        self.scheduled.append((self.polls + polls, event))

    def poll_events(self, since=0):
        self.polls += 1
        self.idle_polls += 1
        for due in [d for d in self.scheduled if d[0] <= self.polls]:
            self.scheduled.remove(due)
            self.post(due[1])
        if self.idle_polls > self.POLL_LIMIT:
            raise AssertionError("a step waited for something the screen never offered")
        return self.seq, [e for e in self.events if e["seq"] > since]

    # what a person can tap
    def on_screen_buttons(self):
        return [b["button_id"] for b in (self.panel or {}).get("buttons", [])]

    def on_screen_objects(self):
        return list(self.objects)


class Operator:
    """Reads the prompts and taps what is on screen: an object first if prefer_objects, else a button."""

    def __init__(self, screen, prefer_objects=False, tap_idless_poi=False, slow_polls=0, prompt_steps=()):
        self.screen = screen
        self.slow_polls = slow_polls        # wait this many polls before tapping...
        self.prompt_steps = prompt_steps    # ...except at these steps, tapped at once
        self.prefer_objects = prefer_objects
        self.tap_idless_poi = tap_idless_poi
        self.title = None
        self.enter = False
        self.used = {}  # step title -> "button:<id>", "object:<id>" or "enter"

    def heard(self, line):
        if "tap one of the POIs" in line:          # the POI step's own check, before its wait
            poi = self.screen.on_screen_objects()[0]["instance_id"]
            self.screen.later({"event_type": "object_tapped", "instance_id": poi}, 2)
            self.used.setdefault("POI check", f"object:{poi}")
        elif "now tap empty background" in line:   # posts nothing; the layer just clears
            self.screen.last_tapped = None
        elif line.startswith("\n["):
            self.title = line.split("] ", 1)[1]
        elif line.startswith("  to go on, tap"):
            self.act()
        elif line.startswith("  press Enter to go on"):
            self.enter = True
            self.used[self.title] = "enter"

    def act(self):
        objects = self.screen.on_screen_objects()
        if self.tap_idless_poi and objects and not any(o.get("instance_id") for o in objects):
            # today's POI layer posts these anyway; the walkthrough must flag it, not move on
            self.screen.post({"event_type": "object_tapped", "instance_id": ""})
        keyed = [o["instance_id"] for o in objects if o.get("instance_id")]
        buttons = self.screen.on_screen_buttons()
        if keyed and (self.prefer_objects or not buttons):
            self.tap({"event_type": "object_tapped", "instance_id": keyed[-1]})
            self.used[self.title] = f"object:{keyed[-1]}"
        elif buttons:
            self.tap({"event_type": "button_press", "button_id": buttons[0]})
            self.used[self.title] = f"button:{buttons[0]}"
        else:
            raise AssertionError(f"step {self.title!r} offers a tap but the screen has nothing to tap")

    def tap(self, event):
        if self.slow_polls and self.title not in self.prompt_steps:
            self.screen.later(event, self.slow_polls)
        else:
            self.screen.post(event)

    def enter_pressed(self):
        pressed, self.enter = self.enter, False
        return pressed


@pytest.fixture
def screen(monkeypatch):
    clock = itertools.count(0.0, 0.5)
    monkeypatch.setattr(ubt, "_sleep", lambda s: None)
    monkeypatch.setattr(ubt, "_clock", lambda: next(clock))
    monkeypatch.setattr(ubt, "find_game_instance", lambda client: None)

    def no_joints(client):
        raise ubt.JointsUnavailable("no joints on this preset")
    monkeypatch.setattr(ubt.JointRig, "locate", staticmethod(no_joints))
    return ScreenClient()


def _run(monkeypatch, screen, operator, tty=True, no_taps=False, timeout=60.0):
    inputs = []

    def hooked_print(*args, **kwargs):
        builtins.print(*args, **kwargs)
        operator.heard(" ".join(str(a) for a in args))
    monkeypatch.setattr(ubt, "print", hooked_print, raising=False)
    monkeypatch.setattr(ubt, "_can_poll_stdin", lambda: tty)
    monkeypatch.setattr(ubt, "_enter_pressed", operator.enter_pressed)
    monkeypatch.setattr(builtins, "input", lambda prompt="": inputs.append(operator.title))
    args = argparse.Namespace(no_taps=no_taps, timeout=timeout, progress_interval=0.0)
    return ubt._walkthrough_steps(screen, args), inputs


def test_every_step_moves_on_from_a_panel_button_except_the_idle_panel(monkeypatch, screen, capsys):
    operator = Operator(screen)
    rc, inputs = _run(monkeypatch, screen, operator)
    assert rc == 0 and inputs == []
    assert [t for t, how in operator.used.items() if how == "enter"] == ["Idle panel"]
    out = capsys.readouterr().out
    assert out.count("round-tripped") == len(operator.used) - 1
    # the progress steps go on from the progress panel's own buttons
    for title in ("Progress advances", "Progress on both scales", "Progress across panels"):
        assert operator.used[title] == "button:pause", title
    # from the POI step on, the panel carries Next
    assert operator.used["A POI tap dims the others"] == f"button:{NEXT_BUTTON}"
    assert operator.used["Wheels drive, then park"] == f"button:{NEXT_BUTTON}"
    assert "walkthrough complete" in out


def test_steps_with_objects_on_screen_move_on_from_a_box_or_poi_tap(monkeypatch, screen, capsys):
    operator = Operator(screen, prefer_objects=True)
    rc, inputs = _run(monkeypatch, screen, operator)
    assert rc == 0 and inputs == []
    by_object = {t: how for t, how in operator.used.items() if how.startswith("object:")}
    assert by_object == {
        "Detected objects": "object:tap_3",
        "Selection moves": "object:tap_3",
        "Objects replaced": "object:knife_7",
        "One object dropped": "object:jug_5",
        "Overlay objects": "object:tap_3",
        "Both cameras at once": "object:knife_7",
    }
    # objects without ids can't be tapped to go on: Next does it
    assert operator.used["Objects without ids"] == f"button:{NEXT_BUTTON}"


def test_the_poi_step_with_a_hud_is_moved_on_by_next_not_by_the_tap_its_check_used(monkeypatch, screen, capsys):
    """The real POI step: its check asks for a POI tap and the background, then the step waits for Next.

    The POI tap its own check consumed is still in the bridge's event buffer, after the step's
    baseline; it must not move the step on, or the Next meant for it would skip the next step.
    """
    monkeypatch.setattr(ubt, "find_game_instance", lambda client: "ROOT")
    monkeypatch.setattr(ubt, "read_pois", lambda client, root, ks=None: screen.pois(root, ks))
    monkeypatch.setattr(ubt, "read_poi_layer", lambda client, root: {"unselected_opacity": 0.4, "visibility": "Visible"})
    monkeypatch.setattr(ubt, "_walkthrough_styles",
                        lambda client, scenario: client.update_panel(ubt.PANEL_SCENARIOS[scenario][0]))
    seen = []
    real = ubt._await_step

    def spy(client, taps, since, timeout=None, sleep=None, clock=None):
        seen.append((operator.title, real(client, taps, since, timeout, sleep, clock)))
        return seen[-1][1]
    monkeypatch.setattr(ubt, "_await_step", spy)
    operator = Operator(screen)
    rc, inputs = _run(monkeypatch, screen, operator)
    out = capsys.readouterr().out
    assert rc == 0 and inputs == []
    assert operator.used["POI check"] == "object:cup_1"
    assert "pass  the tap on cup_1 came from the POI layer" in out and "pass  POI bowl_2 at opacity 0.4" in out
    by_step = dict(seen)
    assert by_step["A POI tap dims the others"] == "button"     # by Next, not by the reused cup_1 tap
    assert by_step["Detected objects"] == "button"
    assert operator.used["A POI tap dims the others"] == f"button:{NEXT_BUTTON}"


@pytest.mark.parametrize("tty, no_taps, panel", [(True, False, "NEXT_PANEL"), (True, True, "ENTER_PANEL"),
                                                 (False, False, "ENTER_PANEL")])
def test_next_is_offered_only_where_the_screen_can_move_a_step_on(monkeypatch, screen, tty, no_taps, panel):
    pushed = []
    real = screen.update_panel
    monkeypatch.setattr(screen, "update_panel", lambda pan: (pushed.append(pan), real(pan)))
    operator = Operator(screen)
    _run(monkeypatch, screen, operator, tty=tty, no_taps=no_taps, timeout=2.0)  # no-tty panel waits run out
    assert getattr(ubt, panel) in pushed
    other = "ENTER_PANEL" if panel == "NEXT_PANEL" else "NEXT_PANEL"
    assert getattr(ubt, other) not in pushed


# The panels whose own buttons are the check: their waits are bounded by --timeout.
CHECKED_STEPS = ("Selection panel", "Progress panel", "Progress gone", "Error panel", "Layout stress",
                 "Button styles", "Style fallback")


def test_steps_that_are_for_looking_wait_for_a_slow_tap(monkeypatch, screen, capsys):
    operator = Operator(screen, slow_polls=30, prompt_steps=CHECKED_STEPS)
    rc, inputs = _run(monkeypatch, screen, operator, timeout=2.0)
    out = capsys.readouterr().out
    assert rc == 0 and inputs == []
    assert "no matching tap" not in out  # 30 polls is ~15 s of fake time, far past --timeout
    assert out.count("round-tripped") == len(operator.used) - 1


def test_a_panel_step_that_gets_no_tap_reports_it_at_the_timeout(monkeypatch, screen, capsys):
    operator = Operator(screen, slow_polls=30, prompt_steps=())  # too slow everywhere
    rc, _ = _run(monkeypatch, screen, operator, timeout=2.0)
    out = capsys.readouterr().out
    assert rc == 0
    missed = [block.split("\n", 1)[0].split("] ", 1)[1] for block in out.split("\n[")[1:]
              if "no matching tap within 2s" in block]
    assert missed == list(CHECKED_STEPS)


def test_a_poi_tap_without_an_id_is_flagged_and_does_not_move_the_step_on(monkeypatch, screen, capsys):
    operator = Operator(screen, tap_idless_poi=True)
    rc, _ = _run(monkeypatch, screen, operator)
    assert rc == 0
    out = capsys.readouterr().out
    step = out.split("] Objects without ids", 1)[1].split("\n[", 1)[0]
    assert "object_tapped with no instance_id" in step
    assert f"OK: '{NEXT_BUTTON}' round-tripped" in step


def test_without_taps_every_step_waits_for_enter_and_nothing_is_polled(monkeypatch, screen):
    operator = Operator(screen)
    rc, inputs = _run(monkeypatch, screen, operator, no_taps=True)
    assert rc == 0
    assert len(inputs) == len(set(inputs)) and "Idle panel" in inputs and "Wheels drive, then park" in inputs
    assert screen.polls == 0 and operator.used == {}


def test_without_a_terminal_panel_steps_still_take_taps_and_the_rest_read_a_line(monkeypatch, screen, capsys):
    operator = Operator(screen)
    # no tty: no prompt is printed for the operator to answer, so press whatever the panel shows
    real_post = screen.post
    real_poll = screen.poll_events

    def poll(since=0):
        if screen.panel and screen.on_screen_buttons() and not any(e["seq"] > since for e in screen.events):
            real_post({"event_type": "button_press", "button_id": screen.on_screen_buttons()[0]})
        return real_poll(since)
    monkeypatch.setattr(screen, "poll_events", poll)
    rc, inputs = _run(monkeypatch, screen, operator, tty=False)
    assert rc == 0
    out = capsys.readouterr().out
    assert "no tty: waiting up to 60s" in out
    # the panels under test round-trip their buttons; everything else reads stdin as before
    assert "Selection panel" not in inputs and "Button styles" not in inputs
    assert "Idle panel" in inputs and "Progress advances" in inputs and "Arm stowed" in inputs


# ── _await_step ──────────────────────────────────────────────────────

def _waiter(monkeypatch, events_at, enter_at=None):
    """A client whose events appear at given poll counts, and a clock of one second per poll."""
    state = {"polls": 0}
    clock = itertools.count(0.0, 1.0)
    monkeypatch.setattr(ubt, "_sleep", lambda s: None)
    monkeypatch.setattr(ubt, "_clock", lambda: next(clock))
    monkeypatch.setattr(ubt, "_enter_pressed", lambda: enter_at is not None and state["polls"] >= enter_at)

    class Client:
        def poll_events(self, since=0):
            state["polls"] += 1
            ready = [dict(e, seq=i + 1) for i, (at, e) in enumerate(events_at) if at <= state["polls"]]
            return len(ready), [e for e in ready if e["seq"] > since]
    return Client()


def test_await_step_ignores_other_buttons_and_takes_its_own(monkeypatch, capsys):
    client = _waiter(monkeypatch, [(1, {"event_type": "button_press", "button_id": "stale"}),
                                   (3, {"event_type": "button_press", "button_id": NEXT_BUTTON})])
    assert ubt._await_step(client, StepTaps(buttons=(NEXT_BUTTON,)), 0) == "button"
    assert "button_press 'stale'" in capsys.readouterr().out


def test_await_step_takes_an_object_tap(monkeypatch):
    client = _waiter(monkeypatch, [(2, {"event_type": "object_tapped", "instance_id": "cup_1"})])
    assert ubt._await_step(client, StepTaps(buttons=(NEXT_BUTTON,), objects=("cup_1",)), 0) == "object"


def test_await_step_without_a_timeout_waits_as_long_as_it_takes(monkeypatch):
    client = _waiter(monkeypatch, [(500, {"event_type": "button_press", "button_id": NEXT_BUTTON})])
    assert ubt._await_step(client, StepTaps(buttons=(NEXT_BUTTON,)), 0, timeout=None) == "button"


def test_await_step_with_a_timeout_reports_the_miss_and_goes_on(monkeypatch, capsys):
    client = _waiter(monkeypatch, [])
    assert ubt._await_step(client, StepTaps(buttons=("a", "b"), checked=True), 0, timeout=5.0) == "timeout"
    assert "no matching tap within 5s (wanted one of ['a', 'b'])" in capsys.readouterr().out


def test_await_step_goes_on_at_enter(monkeypatch):
    client = _waiter(monkeypatch, [], enter_at=3)
    assert ubt._await_step(client, StepTaps(), 0) == "enter"


def test_await_step_does_not_take_an_event_from_before_its_baseline(monkeypatch):
    client = _waiter(monkeypatch, [(1, {"event_type": "button_press", "button_id": NEXT_BUTTON})], enter_at=4)
    assert ubt._await_step(client, StepTaps(buttons=(NEXT_BUTTON,)), 1) == "enter"


def test_step_prompts_name_what_to_tap():
    assert ubt._step_prompt(StepTaps()).startswith("  press Enter to go on")
    prompt = ubt._step_prompt(StepTaps(buttons=(NEXT_BUTTON,), objects=("cup_1", "bowl_2")))
    assert prompt == "  to go on, tap Next on the panel or a box or POI (cup_1, bowl_2) -- or press Enter"
    assert ubt._step_prompt(StepTaps(buttons=("pause", "cancel"))) == \
        "  to go on, tap a button (pause, cancel) -- or press Enter"


# ── Enter on Windows consoles ────────────────────────────────────────

class FakeMsvcrt:
    def __init__(self, keys):
        self.keys = list(keys)

    def kbhit(self):
        return bool(self.keys)

    def getwch(self):
        return self.keys.pop(0)


class FakeTTY:
    def isatty(self):
        return True


def test_enter_is_noticed_on_a_windows_console(monkeypatch):
    monkeypatch.setattr(ubt.os, "name", "nt")
    monkeypatch.setattr(ubt.sys, "stdin", FakeTTY())
    monkeypatch.setattr(ubt, "msvcrt", FakeMsvcrt("x\r"))
    assert ubt._can_poll_stdin() and ubt._enter_pressed()
    monkeypatch.setattr(ubt, "msvcrt", FakeMsvcrt("xy"))
    assert not ubt._enter_pressed()
    monkeypatch.setattr(ubt, "msvcrt", None)
    assert not ubt._can_poll_stdin()


def test_captured_or_piped_stdin_cannot_be_polled(monkeypatch):
    class Piped:
        def isatty(self):
            return False
    monkeypatch.setattr(ubt.sys, "stdin", Piped())
    assert not ubt._can_poll_stdin() and not ubt._enter_pressed()
