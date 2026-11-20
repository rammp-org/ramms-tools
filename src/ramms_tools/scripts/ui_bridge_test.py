#!/usr/bin/env python3
"""
Exercise the RammsBridgeReceiver interfaces that the RAMMP UI bridge drives:
panels/buttons, detected-object and world-state views, the camera feed, the
RMSS camera streams, and the UI events that come back.

Usage:
    ramms-ui-bridge-test discover                 # what the preset exposes + live stream IDs
    ramms-ui-bridge-test selftest                 # non-interactive transport checks
    ramms-ui-bridge-test panel --scenario selection
    ramms-ui-bridge-test objects --source scene --select cup_1
    ramms-ui-bridge-test worldstate
    ramms-ui-bridge-test camera open
    ramms-ui-bridge-test stream --seconds 10      # synthetic RMSS camera frames
    ramms-ui-bridge-test events --follow          # watch taps as you click
    ramms-ui-bridge-test events --expect probe:ok --timeout 30
    ramms-ui-bridge-test styles                   # every button style, read back and checked
    ramms-ui-bridge-test styles --scenario style-fallback   # unknown / differently-cased styles
    ramms-ui-bridge-test pois --tap               # POIs, and a tap dimming the others
    ramms-ui-bridge-test markers                  # world markers derived from 2D + depth (editor PIE)
    ramms-ui-bridge-test joints                   # robot display: joints, monitor, status bar
    ramms-ui-bridge-test joints --check           # stow/activate/drive/park, all read back
    ramms-ui-bridge-test joints --set link_1=40   # by name; every other joint held
    ramms-ui-bridge-test walkthrough              # guided pass over everything

Requires the UE editor running with Remote Control enabled (HTTP on 30010) and
a bridge preset loaded -- i.e. a level containing RammsBridgeReceiver.
The preset is discovered automatically; --preset pins one.

WHAT THIS CAN AND CANNOT CHECK
------------------------------
Automatic: that every call reaches UE with the parameter names the preset
actually declares, and that taps come back through PollUIEvents with the exact
IDs that were pushed. Those are the two things that broke silently before -- a
mis-named parameter is accepted with HTTP 200 and arrives as an empty string.

Also automatic: the robot display's joints. Joint targets, the joint-motion
monitor's verdict (arm Stowed/Active, wheels moving) and the status bar's text
are all readable as properties, so `joints --check` and selftest's joints section
assert them. Writes go only to a running game -- PIE, or a packaged or -game build:
in the editor with nothing playing, the editor's own level answers on the same
paths and would be written instead, so selftest skips the joints and says so.

Also automatic, in a running game: what the panel's buttons draw and what the
POI layer shows. Each button's id, label, style, label text, label opacity and
brush tint, and each POI's object and opacity, are readable widget properties, so
`styles` checks that every button shows its own label, that unknown and
differently-cased style names fall back as RAMMS_Button's switch does, and that
its styles are told apart; `pois` checks one POI per object and, after a tap
(`--tap`), that the others dim to the layer's UnselectedPOIOpacity.

Not automatic: anything else about how the HUD *looks* -- layout, colours as a
person sees them, the panel-level style (RAMMS_Panel does not draw it), world
state and the camera feed. Those scenarios print what you should see and, where
it matters, wait for a tap and confirm the ID round-trips.

The walkthrough moves on from the screen as well as from Enter: a panel step on
a press of one of that panel's buttons, and every later step on the panel's Next
button or a tap on any box or POI it shows. Each press or tap round-trips too.
"""

import argparse
import json
import math
import os
import re
import select
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import NamedTuple

try:
    import msvcrt  # Windows: lets Enter be noticed without blocking
except ImportError:
    msvcrt = None

DEFAULT_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 30010
DEFAULT_STREAM_PORT = 30030
# Discover the preset rather than naming one. Projects disagree: RammpUI binds
# the bridge on its own Content/RCPS, while the Ramms project has only
# RCP_RammsBridge, so any hardcoded default is wrong somewhere. --preset pins it.
PRESET = "auto"

# What a preset must expose, and have bound, to be worth talking to.
REQUIRED_FUNCTIONS = ("UpdatePanel", "UpdateDetectedObjects", "PollUIEvents")
SETTINGS_PATH = "/Script/RammsUI.Default__RammsCameraStreamSettings"

# Fallback only -- the real labels/arguments are discovered from the preset.
# Note the endpoint wants the exposed LABEL ("Poll UIEvents"), not the C++ name.
FALLBACK_LABELS = {
    "UpdatePanel": "Update Panel",
    "UpdateDetectedObjects": "Update Detected Objects",
    "UpdateWorldState": "Update World State",
    "OpenCameraFeed": "Open Camera Feed",
    "CloseCameraFeed": "Close Camera Feed",
    "PollUIEvents": "Poll UIEvents",
}


# ── joints ───────────────────────────────────────────────────────────
#
# The RAMMP display's arm and wheels are driven by one call: SetAllJointTargets(Values) on its
# URammsSkeletalPoseComponent, exposed on the preset as "Set All Joint Targets". Values is
# POSITIONAL over the component's Joints array (53 entries on the RAMMP display: wheels at 2
# and 9, arm link_0..link_6 at 46-52, degrees for revolute joints) -- so every vector here is
# built BY NAME from the component's own list, never from hardcoded indices.
#
# What reacts: URammsJointMotionMonitorComponent classifies each write (arm Stowed/Active
# against its stow pose; wheels moving while their targets change), and the HUD's status bar
# shows the result (arm "Stowed"/"Active", chair "DRIVING"/"PARKED"). All three are readable as
# properties over Remote Control, so these checks are automatic rather than eyes-only.

JOINT_FUNCTION = "SetAllJointTargets"
# Remote Control's preset description lists NO arguments for SetAllJointTargets: it omits
# parameters flagged CPF_OutParm, and a `const TArray<double>&` parameter carries that flag.
# Taking the empty list at its word would refuse the one parameter the function has. The name
# is pinned here instead -- and every send is confirmed by reading the targets back, because a
# wrong name is accepted with HTTP 200 and arrives as an empty array.
KNOWN_ARGUMENTS = {JOINT_FUNCTION: ["Values"]}
# What a dangling entry should be rebound to, for the ones that are not the receiver's.
REBIND_TARGETS = {
    JOINT_FUNCTION: "the RammsSkeletalPose component of the robot display (BP_RAMMP_RemoteDisplay*)",
    "UpdateSystemState": "the robot display actor (BP_RAMMP_RemoteDisplay*)",
}
MONITOR_COMPONENT = "JointMotionMonitor"
# Which engine is running, told by which engine object exists. The editor's (UEditorEngine)
# PlayWorld names the live play world -- "/Game/Maps/UEDPIE_0_Map_UI.Map_UI" -- or is empty when
# nothing is playing, which matters because the editor's own level then answers on the very paths
# a game would. A game, packaged or `UnrealEditor -game`, has GameEngine_0 and no editor level: the
# bound path is the live robot. (An editor-only settings CDO is NOT a test for the editor: the
# editor binary loads UnrealEd in -game too.)
EDITOR_ENGINE = ("/Engine/Transient.UnrealEdEngine_0", "PlayWorld")
GAME_ENGINE = ("/Engine/Transient.GameEngine_0", "bUseFixedFrameRate")
# The HUD's status bar, under whichever engine owns the game instance. The instance number
# changes every play session and Remote Control has no object search, so it is probed.
STATUS_BAR_PATH = ("/Engine/Transient.{engine}:GameInstance_{n}.W_RAMMS_UI_C_0.WidgetTree_0."
                   "RAMMS_StatusBar.WidgetTree_0.{widget}")
STATUS_BAR_ENGINES = ("UnrealEdEngine_0", "GameEngine_0")
STATUS_BAR_INSTANCES = 1024  # one per play session this editor has run; all probed in one request


def _percent(text):
    """A progress pct, on either scale the UI accepts: 0.0-1.0 or 0-100.

    USER_INPUT_JSON_SCHEMA.md 6.4 states the convention as 0.0-1.0 and records
    fridge_ui_demo as yielding 0.0-100.0 for the widget it targets.
    RammsBridgeSubsystem::PanelProgressFraction reconciles both: over 1 is a
    percentage, 1 or less is a fraction. So 0.65 and 65 mean the same bar, and
    1.0 means full rather than one percent.
    """
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number")
    if not 0.0 <= value <= 100.0:
        raise argparse.ArgumentTypeError("--pct is 0.0-1.0 or 0-100")
    return value


def _positive_int(text):
    """A count of objects to push: at least one. 0 would be a clear wearing a push's
    message, and a negative value silently drops from the END of the set instead."""
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not an integer")
    if value < 1:
        raise argparse.ArgumentTypeError("--count must be 1 or more; use --clear to push nothing")
    return value


def _positive_float(text):
    """argparse type: --fps 0 would divide by zero when spacing the channels."""
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be greater than 0")
    return value


class BridgeError(RuntimeError):
    pass


class BridgeClient:
    """Remote Control HTTP client for the bridge preset (discovered, or --preset)."""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_HTTP_PORT, timeout=8.0,
                 preset=PRESET, verbose=False):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout
        self.preset = preset
        self.verbose = verbose
        self._labels = {}    # C++ name -> exposed label
        self._arguments = {}  # C++ name -> [argument names]
        self._owners = {}    # C++ name -> [bound object paths]
        self._duplicates = {}  # C++ name -> [every label exposing it]
        self._load_failed = None  # remembered reason, so we stop retrying

    # ── transport ────────────────────────────────────────────────────

    def _request(self, method, endpoint, payload=None):
        url = f"{self.base}{endpoint}"
        data = json.dumps(payload).encode() if payload is not None else None
        req = urllib.request.Request(url, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode()
        except urllib.error.URLError as exc:
            raise BridgeError(f"{method} {endpoint} failed: {exc}") from exc
        if not body.strip():
            return None
        try:
            # strict=False: Remote Control puts raw newlines in some errorMessages. Strict parsing
            # rejects the whole body, which for /remote/batch silently loses every response in it.
            parsed = json.loads(body, strict=False)
        except json.JSONDecodeError:
            return body
        # Remote Control reports failures in-band with HTTP 200.
        if isinstance(parsed, dict) and "errorMessage" in parsed:
            raise BridgeError(parsed["errorMessage"].strip())
        return parsed

    # ── preset introspection ─────────────────────────────────────────

    def _preset_entries(self, name):
        """Exposed functions of one preset: {cpp name: [(label, args, owners)]}."""
        data = self._request("GET", f"/remote/preset/{name}")
        if not isinstance(data, dict):
            raise BridgeError(f"preset {name}: unexpected response {data!r:.80}")
        preset = data.get("Preset", data)
        entries = {}
        for group in preset.get("Groups", []):
            for fn in group.get("ExposedFunctions", []):
                underlying = fn.get("UnderlyingFunction", {})
                cpp = underlying.get("Name")
                if not cpp:
                    continue
                # A missing OwnerObjects key is "unknown", not "bound to
                # nothing" -- only an empty list means orphaned.
                owners = (None if "OwnerObjects" not in fn
                          else [o.get("Path", "") for o in (fn["OwnerObjects"] or [])])
                entries.setdefault(cpp, []).append((
                    fn.get("DisplayName") or cpp,
                    [a.get("Name") for a in underlying.get("Arguments", [])],
                    owners,
                ))
        return preset, entries

    def discover_preset(self):
        """The preset that actually serves this API, by inspection.

        Presence is not enough: an orphaned entry is still listed, and calling
        it returns HTTP 200 while reaching nothing. A preset only counts if the
        functions are there AND bound.
        """
        data = self._request("GET", "/remote/presets")
        rows = (data or {}).get("Presets", []) if isinstance(data, dict) else []
        if not rows:
            raise BridgeError("no Remote Control presets on this editor")
        rejected = []
        for row in rows:
            name = row.get("Name")
            if not name:
                continue
            try:
                _, entries = self._preset_entries(name)
            except BridgeError as exc:
                rejected.append(f"{name}: unreadable ({exc})")
                continue
            missing = [f for f in REQUIRED_FUNCTIONS if f not in entries]
            orphaned = [f for f in REQUIRED_FUNCTIONS
                        if f in entries and not any(e[2] for e in entries[f])]
            if missing:
                rejected.append(f"{name}: missing {missing}")
            elif orphaned:
                rejected.append(f"{name}: {orphaned} bound to nothing")
            else:
                return name
        raise BridgeError(
            "no preset exposes the bridge API with its bindings intact:\n  "
            + "\n  ".join(rejected)
            + "\nPass --preset to pin one, or rebind it to the receiver in the open level.")

    def load_preset(self):
        """Map each C++ function name to its exposed label, arguments and owners."""
        if self._load_failed:
            # Do not re-attempt on every call: one unreachable editor would
            # otherwise cost three full timeouts per command.
            raise BridgeError(self._load_failed)
        try:
            if self.preset == "auto":
                self.preset = self.discover_preset()
            preset, entries = self._preset_entries(self.preset)
        except BridgeError as exc:
            self._load_failed = str(exc)
            raise

        self._labels, self._arguments, self._owners, self._duplicates = {}, {}, {}, {}
        for name, found in entries.items():
            self._duplicates[name] = [label for label, _, _ in found]
            # A preset can expose one function more than once, under different
            # labels and bindings -- re-exposing after a level move leaves the
            # stale entry and adds "Name (1)". Prefer one that is bound; a blind
            # last-wins would pick whichever was serialised last.
            chosen = next((e for e in found if e[2]), found[-1])
            self._labels[name], self._arguments[name], self._owners[name] = chosen
        if not self._labels:
            raise BridgeError(f"preset {self.preset} exposes no functions")
        return preset

    def label_for(self, function):
        if not self._labels:
            try:
                self.load_preset()
            except BridgeError:
                pass
        return self._labels.get(function, FALLBACK_LABELS.get(function, function))

    def call(self, function, **parameters):
        """Call a preset function, checking parameter names against the preset.

        UE silently ignores parameters it does not recognise, so an unknown
        name shows up as an empty string on the UE side rather than an error.
        Catch that here instead.

        Matching is case-INSENSITIVE, though: PanelJSON, PanelJson and
        paneljson all land (verified against a live editor), so only a genuinely
        unrecognised name is a problem. Compare case-insensitively or this
        refuses calls that would have worked.
        """
        declared = self._arguments.get(function)
        if declared is None and not self._labels:
            try:
                self.load_preset()
                declared = self._arguments.get(function)
            except BridgeError:
                declared = None
        if declared == [] and function in KNOWN_ARGUMENTS:
            declared = KNOWN_ARGUMENTS[function]
        if declared is not None:
            known = {name.lower() for name in declared if name}
            unknown = sorted(n for n in parameters if n.lower() not in known)
            if unknown:
                raise BridgeError(
                    f"{function} does not take {unknown}; it declares {declared}. "
                    "UE would accept the call with HTTP 200 and receive an empty "
                    "value for the real parameter."
                )
        self.check_bound(function)
        label = urllib.parse.quote(self.label_for(function))
        payload = {"Parameters": parameters, "GenerateTransaction": False}
        if self.verbose:
            print(f"  -> {function}({', '.join(parameters)})")
        result = self._request("PUT", f"/remote/preset/{self.preset}/function/{label}", payload)
        returned = (result or {}).get("ReturnedValues") or [{}]
        return returned[0]

    def owners_for(self, function):
        """Objects the exposed function is bound to, or None if unknown."""
        if not self._labels:
            try:
                self.load_preset()
            except BridgeError:
                return None
        return self._owners.get(function)


    def check_bound(self, function):
        """Raise if the function is exposed but bound to nothing.

        An unbound entry is the worst failure this interface has: Remote
        Control answers HTTP 200 with an empty ReturnedValues and the call
        reaches no object at all, so every push looks like it worked and the UI
        never changes. It happens whenever the level holding the receiver is
        moved or replaced -- the preset keeps a binding to the old path.
        """
        owners = self.owners_for(function)
        if owners is not None and not owners:
            target = REBIND_TARGETS.get(function, "the RammsBridgeReceiver")
            raise BridgeError(
                f"{function} is exposed on preset {self.preset} but bound to no "
                "object, so the call would return HTTP 200 and reach nothing. "
                f"Rebind it to {target} in the level that is currently open, then "
                "re-run this command -- the preset is read once per run."
            )

    def unbound_functions(self):
        """Every exposed function bound to nothing, or None if that is unknown.

        None rather than [] when the preset cannot be read: an empty list is an
        all-clear, and a caller should not get one from a failed lookup.
        """
        if not self._labels:
            try:
                self.load_preset()
            except BridgeError:
                return None
        return sorted(name for name, owners in self._owners.items()
                      if owners is not None and not owners)

    def call_object(self, object_path, function, **parameters):
        """Call a UFUNCTION on one object by path, outside any preset.

        Subject to the project's Remote Control allow-list (Config/DefaultRemoteControl.ini):
        a function that is not on it comes back as an errorMessage, which _request raises.
        """
        payload = {"objectPath": object_path, "functionName": function,
                   "parameters": parameters, "generateTransaction": False}
        if self.verbose:
            print(f"  -> {object_path}.{function}({', '.join(parameters)})")
        return self._request("PUT", "/remote/object/call", payload) or {}

    def read_many(self, reads):
        """Read several (object_path, property) pairs in ONE request: {index: value} for those found.

        /remote/batch runs the reads together, so probing hundreds of candidate paths costs one
        round trip (measured: 512 in 0.18 s) instead of one engine tick each -- a backgrounded
        editor answers single reads at a few dozen a second.
        """
        requests = [{"RequestId": i, "URL": "/remote/object/property", "Verb": "PUT",
                     "Body": {"objectPath": path, "propertyName": prop, "access": "READ_ACCESS"}}
                    for i, (path, prop) in enumerate(reads)]
        body = self._request("PUT", "/remote/batch", {"Requests": requests}) or {}
        found = {}
        for response in body.get("Responses", []) if isinstance(body, dict) else []:
            value = response.get("ResponseBody")
            if response.get("ResponseCode") == 200 and isinstance(value, dict):
                found[response.get("RequestId")] = value.get(reads[response.get("RequestId")][1])
        return found

    def get_setting(self, property_name, object_path=SETTINGS_PATH):
        payload = {"objectPath": object_path, "propertyName": property_name,
                   "access": "READ_ACCESS"}
        result = self._request("PUT", "/remote/object/property", payload)
        return (result or {}).get(property_name)

    # ── the six bridge functions ─────────────────────────────────────

    def update_panel(self, panel: dict):
        return self.call("UpdatePanel", PanelJSON=json.dumps(panel))

    def update_detected_objects(self, payload: dict):
        return self.call("UpdateDetectedObjects", DetectedObjectsJSON=json.dumps(payload))

    def update_world_state(self, payload: dict):
        return self.call("UpdateWorldState", WorldStateJSON=json.dumps(payload))

    def open_camera_feed(self):
        return self.call("OpenCameraFeed")

    def close_camera_feed(self):
        return self.call("CloseCameraFeed")

    def poll_events(self, since=0):
        result = self.call("PollUIEvents", SinceSequence=since)
        latest = int(result.get("LatestSequence", 0))
        events = []
        for raw in result.get("Events", []):
            try:
                events.append(json.loads(raw))
            except json.JSONDecodeError:
                print(f"  ! unparseable event: {raw!r}")
        return latest, events


# ── payload builders (docs/USER_INPUT_JSON_SCHEMA.md) ────────────────

# Button styles. The bridge contract (rammp-UI-bridge docs/USER_INPUT_JSON_SCHEMA.md §6.3) defines
# normal | warning | error, and the Orchestrator sends its Cancel buttons as "warning". RAMMS_Button
# (ramms-ui 4769575, read from its graph 2026-11-20) recognises valid | invalid | unavailable | cancel
# | error, case-insensitively, and draws anything else -- normal, warning, absent, unknown -- as valid.
CONTRACT_BUTTON_STYLES = ("normal", "warning", "error")
UI_BUTTON_STYLES = ("valid", "invalid", "unavailable", "cancel", "error")
BUTTON_STYLES = CONTRACT_BUTTON_STYLES + tuple(s for s in UI_BUTTON_STYLES if s not in CONTRACT_BUTTON_STYLES)


def ui_button_style(style):
    """The RAMMS_Button style a sent style selects: its own when it is one, else the fallback, valid."""
    key = (style or "").lower()
    return key if key in UI_BUTTON_STYLES else "valid"


def button(button_id, label, style=None):
    b = {"button_id": button_id, "label": label}
    if style:
        b["style"] = style
    return b


def panel(title="", message="", buttons=None, progress=None, style=None):
    """An InteractionPanel. `buttons` is always present, even when empty.

    `progress` and `style` are omitted entirely when unset rather than sent as
    null -- UE-side code uses key-existence checks.
    """
    p = {"title": title, "message": message, "buttons": list(buttons or [])}
    if progress is not None:
        p["progress"] = progress
    if style is not None:
        p["style"] = style
    return p


# The values an advancing run walks through, on the 0-100 scale this UI expects (see
# _percent). 0.0 is deliberately first: it is the value most likely to be mistaken for
# "no progress" by anything that infers the flag from the payload instead of from the
# key's presence.
PROGRESS_STEPS = (0.0, 30.0, 65.0, 100.0)


# The same bar expressed on both scales. PanelProgressFraction must read each pair
# identically -- over 1 is a percentage, 1 or less a fraction -- so a panel that only
# handles one scale shows a mismatched pair here and nowhere else.
PROGRESS_SCALE_PAIRS = ((0.25, 25.0), (0.65, 65.0), (1.0, 100.0))


def progress_panel(pct, phase="opening", title="fridge_ui_demo: open",
                   message="opening door", pausable=True, buttons=None):
    """A progress panel at one value. `pct` is 0-100 for this UI, not a 0.0-1.0 fraction.

    The defaults keep everything except `pct` identical from call to call, which is
    what makes an advancing run a test of the bar updating in place rather than of
    the panel being rebuilt around it.
    """
    return panel(
        title=title,
        message=message,
        buttons=buttons if buttons is not None
        else [button("pause", "Pause", "normal"), button("cancel", "Cancel", "warning")],
        progress={"phase": phase, "pct": pct, "message": message, "pausable": pausable},
    )


# Distinct panels that each carry their own progress: different phase, title, message,
# buttons and fraction. A bar that keeps a previous panel's value, or that only tracks
# the first panel it saw, shows up here and not in an advancing run of one panel.
PROGRESS_PANELS = (
    progress_panel(15.0, phase="unlatching", title="fridge_ui_demo: open",
                   message="unlatching"),
    progress_panel(55.0, phase="swinging", title="fridge_ui_demo: open",
                   message="swinging door wide"),
    progress_panel(80.0, phase="swinging", title="fridge_ui_demo: open",
                   message="paused", pausable=False,
                   buttons=[button("resume", "Resume", "normal"),
                            button("cancel", "Cancel", "warning")]),
    progress_panel(45.0, phase="reaching", title="arm_demo: reach",
                   message="reaching for the handle"),
)


def detected_object(instance_id, perception_class, box, centroid=None, source="scene"):
    """One tappable object.

    `box`/`centroid` go on the wire as JSON objects with X/Y/Z/W keys, not as
    bare arrays: they land in FVector4/FVector2D properties, and UE's JSON
    struct converter rejects an array outright.
    """
    x_min, y_min, x_max, y_max = box
    obj = {
        "instance_id": instance_id,
        "perception_class": perception_class,
        "box": {"X": x_min, "Y": y_min, "Z": x_max, "W": y_max},
        "source": source,
    }
    cx, cy = centroid if centroid else ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)
    obj["centroid"] = {"X": cx, "Y": cy}
    return obj


def detected_objects_payload(objects, selected_instance_id="", stream_id=""):
    return {
        "stream_id": stream_id,
        "selected_instance_id": selected_instance_id,
        "objects": objects,
    }


# ── panel scenarios ──────────────────────────────────────────────────

PANEL_SCENARIOS = {
    "idle": (
        panel(message="Select an object or action to begin."),
        "A message with no buttons at all.",
    ),
    "selection": (
        panel(
            title="fridge_1",
            message="What would you like to do?",
            buttons=[
                button("action:fridge_ui_demo:open", "Open"),
                button("action:fridge_ui_demo:close", "Close"),
                button("deselect", "Deselect"),
            ],
        ),
        "Three buttons, none styled -- absent style must render as normal.",
    ),
    "progress": (
        panel(
            title="fridge_ui_demo: open",
            message="opening door",
            buttons=[button("pause", "Pause", "normal"), button("cancel", "Cancel", "warning")],
            progress={"phase": "opening", "pct": 40.0, "message": "opening door",
                      "pausable": True},
        ),
        "A progress bar visibly at 40% plus Pause/Cancel. pct is 0-100 for this UI,\n"
        "          not the schema's nominal 0.0-1.0 fraction -- RAMMS_Panel divides it. A bar\n"
        "          that appears but looks EMPTY means a fraction was sent: 0.4 is 0.4%.\n"
        "          NO BAR AT ALL is the known failure: the schema marks progress optional\n"
        "          by omitting the key, so FInteractionPanel's hasprogress is never sent\n"
        "          and Json String To Struct leaves it false. The panel must set it from\n"
        "          the key's presence (RammsBridgeSubsystem::PanelJsonHasProgress).",
    ),
    "paused": (
        panel(
            title="fridge_ui_demo: open",
            message="paused",
            buttons=[button("resume", "Resume", "normal"), button("cancel", "Cancel", "warning")],
            progress={"phase": "opening", "pct": 40.0, "message": "paused", "pausable": False},
        ),
        "Resume replaces Pause; the bar should stay where it was.",
    ),
    "error": (
        panel(
            title="Gripper stalled",
            message="The gripper did not reach its target. Retry or cancel.",
            buttons=[button("retry", "Retry", "normal"), button("cancel", "Cancel", "error")],
            style="error",
        ),
        "Retry looks normal and Cancel error-styled. The panel's own style is sent\n"
        "          too, but RAMMS_Panel does not draw it: the panel looks like any other.\n"
        "          Its only effect is that a style change under the same title replays\n"
        "          the entrance animation, as a new panel would.",
    ),
    "styles": (
        panel(
            title="Button styles",
            message="One button per style, each labelled with the style it was sent.",
            buttons=[button("style:absent", "absent")]
            + [button(f"style:{name}", name, name) for name in BUTTON_STYLES],
        ),
        "Every button reads its OWN label (the style name). RAMMS_Button today:\n"
        "          absent, normal, warning, valid: blue, white label\n"
        "          invalid: amber, dark label;  error: flat dark navy, white label\n"
        "          unavailable: blue, label at half opacity (still clickable)\n"
        "          cancel: KNOWN FAILURE -- drawn like invalid and reading \"OPEN\", not\n"
        "          \"cancel\": SetStyle's cancel branch is not connected to SetStyle_Cancel.\n"
        "          Also check: warning looks the same as normal, though the contract\n"
        "          uses it for the Orchestrator's Cancel buttons. `styles` checks this.",
    ),
    "style-fallback": (
        panel(
            title="Style fallback",
            message="Unknown styles fall back to valid; style names ignore case.",
            buttons=[button("fallback:valid", "valid", "valid"),
                     button("fallback:unavailable", "unavailable", "unavailable"),
                     button("fallback:error", "error", "error"),
                     button("fallback:bogus", "bogus", "bogus"),
                     button("fallback:VALID", "VALID", "VALID"),
                     button("fallback:Unavailable", "Unavailable", "Unavailable"),
                     button("fallback:Error", "Error", "Error"),
                     {"button_id": "fallback:empty", "label": "empty", "style": ""}],
        ),
        "The first three are the references. bogus, VALID and empty look like valid\n"
        "          (blue); Unavailable looks like unavailable and Error like error --\n"
        "          style names ignore case.",
    ),
    "stress": (
        panel(
            title="A title that is quite a lot longer than any real panel would ever use",
            message=("Long body text to check wrapping and clipping. " * 6
                     + "Unicode: café — ½° ✓ 日本語"),
            buttons=[button(f"many:{i}", f"Button {i}", BUTTON_STYLES[i % len(BUTTON_STYLES)])
                     for i in range(8)],
        ),
        "Long text and 8 buttons in every style -- check wrapping, clipping and layout.",
    ),
    "empty": (
        panel(),
        "Everything blank: empty title, empty message, no buttons. Should not crash.",
    ),
}


def scenario_buttons(name):
    return [b["button_id"] for b in PANEL_SCENARIOS[name][0]["buttons"]]


# ── object scenarios ─────────────────────────────────────────────────

# Normalized (0-1) boxes, which is what the widget wants by default.
#
# URammsCameraWidget::GetDetectionCoordinateScale treats every coordinate <= 1.0
# as already normalized and uses it as-is; anything larger is taken as PIXELS of
# the current frame and divided by that frame's size -- or by
# FallbackDetectionImageSize, (640, 480), before any frame has arrived. So a
# pixel-space box drawn for a 1280x720 feed lands at twice its intended position
# against the fallback and falls off the right edge entirely. Normalized boxes
# show up in the same place whatever is (or isn't) streaming.
NORMALIZED_SAMPLES = [
    ("cup_1", "cup", (0.08, 0.12, 0.34, 0.55), None),
    ("bowl_2", "bowl", (0.40, 0.30, 0.68, 0.78), None),
    ("tap_3", "faucet", (0.74, 0.10, 0.92, 0.48), (0.88, 0.42)),
]

# The same three in the pixel space of a 1280x720 feed, for --pixels.
PIXEL_SAMPLES = [
    ("cup_1", "cup", (102, 86, 435, 396), None),
    ("bowl_2", "bowl", (512, 216, 870, 562), None),
    ("tap_3", "faucet", (947, 72, 1178, 346), (1126, 302)),
]

# A disjoint set: no instance_id here appears in NORMALIZED_SAMPLES. Pushing this
# after that one is what catches a layer that adds without removing -- the ids have
# to differ, or reusing a marker and leaking one look the same on screen.
NORMALIZED_SAMPLES_B = [
    ("jug_5", "jug", (0.12, 0.40, 0.38, 0.86), None),
    ("plate_6", "plate", (0.44, 0.52, 0.72, 0.90), None),
    ("knife_7", "knife", (0.60, 0.08, 0.95, 0.30), (0.78, 0.20)),
]

# Objects with no instance_id: they draw, but nothing can key a marker on them, so
# they exercise the unkeyed path (reused in order, surplus removed) and a tap on one
# has no id to send.
UNKEYED_SAMPLES = [
    ("", "unknown", (0.20, 0.20, 0.40, 0.45), None),
    ("", "unknown", (0.55, 0.55, 0.78, 0.82), None),
]

SAMPLE_SETS = {
    "a": NORMALIZED_SAMPLES,
    "b": NORMALIZED_SAMPLES_B,
    "unkeyed": UNKEYED_SAMPLES,
}


def sample_objects(source, pixels=False, variant="a", count=None):
    """Tappable objects from one sample set, normalized unless `pixels` asks otherwise.

    `count` takes only the first N, to check that dropping an object removes its
    marker rather than leaving it behind.
    """
    if variant not in SAMPLE_SETS:
        raise KeyError(f"unknown sample set {variant!r}; try one of {sorted(SAMPLE_SETS)}")
    if pixels and variant != "a":
        raise ValueError(f"there is no pixel-space sample set for {variant!r}, only for 'a'")
    samples = PIXEL_SAMPLES if pixels else SAMPLE_SETS[variant]
    if count is not None:
        samples = samples[:count]
    return [detected_object(iid, cls, box, centroid, source)
            for iid, cls, box, centroid in samples]


# ── commands ─────────────────────────────────────────────────────────

# ── joints: pure helpers ─────────────────────────────────────────────

def pie_path(path, instance=0):
    """The play-in-editor copy of an editor-world object path.

    PIE duplicates the level into a package prefixed UEDPIE_<n>_; the object names inside it are
    unchanged. A preset binding names the EDITOR's level, so reading that path during play reads
    the editor's copy -- which never ticks. Measured: it answered "Stowed" while the running game
    said "Active". Paths that are not in a level package come back unchanged.
    """
    package, sep, rest = path.partition(".")
    head, slash, short = package.rpartition("/")
    # Only an object INSIDE a map ("<package>.<map>:PersistentLevel.<actor>") has a play copy;
    # a class default or an asset ("/Script/Module.Default__X") does not.
    if not sep or ":" not in rest or not slash or short.startswith("UEDPIE_"):
        return path
    return f"{head}/UEDPIE_{instance}_{short}{sep}{rest}"


def play_copy(path, play_world):
    """`path` moved into the play world the editor reports, or None if it is not that map's.

    play_world is UEditorEngine::PlayWorld, e.g. "/Game/Maps/UEDPIE_0_Map_UI.Map_UI": its instance
    number is used as reported rather than assumed to be 0.
    """
    match = re.fullmatch(r"(.*/)UEDPIE_(\d+)_([^/.]+)\.\3", play_world or "")
    if not match:
        return None
    live = pie_path(path, int(match.group(2)))
    return live if live.startswith(f"{match.group(1)}UEDPIE_{match.group(2)}_{match.group(3)}.") else None


def plain_text(rich):
    """A rich-text block's visible text: '<status-default>Stowed</>' -> 'Stowed'."""
    return re.sub(r"<[^>]*>", "", rich or "").strip()


def joint_index(joints):
    """JointName -> position in the Values array."""
    return {j.get("JointName"): i for i, j in enumerate(joints)}


def with_joints(targets, index, values):
    """targets with the named joints replaced and every other joint held.

    Exact names only: the gripper's own joints are called link_0_l, link_3_r and so on, and a
    loose match would move the gripper when the arm was meant.
    """
    missing = sorted(name for name in values if name not in index)
    if missing:
        raise BridgeError(f"the display has no joint(s) {missing}")
    out = [float(v) for v in targets]
    for name, value in values.items():
        out[index[name]] = float(value)
    return out


def within_limits(joint, value):
    """Whether a target would be applied as sent (a joint with enforced limits clamps it)."""
    if not joint.get("bEnforceLimits"):
        return True
    return float(joint.get("MinValue", value)) <= value <= float(joint.get("MaxValue", value))


def active_offset(joints, index, arm_names, stowed, tolerance):
    """One arm joint moved far past the stow tolerance, in a direction its limits allow.

    Far past rather than just past, so a joint that drifts or a rounding difference cannot
    leave the monitor straddling the boundary; and unclamped, so the readback compares what
    was sent with what was applied.
    """
    step = max(30.0, 6.0 * float(tolerance))
    for name, base in zip(arm_names, stowed):
        joint = joints[index[name]]
        for value in (float(base) + step, float(base) - step):
            if within_limits(joint, value):
                return {name: value}
    raise BridgeError("no arm joint can move past the stow tolerance without hitting its limits")


# ── joints: the display, found through its preset binding ───────────

class JointsUnavailable(BridgeError):
    """Joints cannot be checked here -- a reason to skip, not to fail.

    The preset does not expose joint targets at all, or the editor is not playing (so the only
    copy of the robot that answers is the one the saved level holds).
    """


class JointRig:
    """The robot display's pose component, monitor and status bar, resolved for the live world."""

    def __init__(self, client, pose_path, world):
        self.client = client
        self.pose_path = pose_path
        self.world = world  # "play session" (editor PIE) | "game" (packaged or -game)
        self.actor_path = pose_path.rpartition(".")[0]
        self.monitor_path = f"{self.actor_path}.{MONITOR_COMPONENT}"
        self.joints = self.read_joints()
        if not self.joints:
            raise BridgeError(f"{pose_path} has no joints")
        self.index = joint_index(self.joints)
        self.monitor = self._read_monitor()
        self._status_root = None  # (engine, game instance number) once found

    # ── locating ─────────────────────────────────────────────────

    @classmethod
    def locate(cls, client):
        if not client._labels:
            client.load_preset()
        if JOINT_FUNCTION not in client._labels:
            raise JointsUnavailable(f"preset {client.preset} does not expose {JOINT_FUNCTION}")
        client.check_bound(JOINT_FUNCTION)
        owners = client.owners_for(JOINT_FUNCTION) or []
        if not owners:
            raise BridgeError(f"preset {client.preset} does not report what {JOINT_FUNCTION} is bound to")
        bound = owners[0]
        play_world = _read(client, *EDITOR_ENGINE)
        if play_world is not _ABSENT:
            # The editor. Write only into its play world: its own level answers on the very path
            # the binding names, and a write there lands in the actor whose values the map saves.
            if not play_world:
                raise JointsUnavailable(
                    "the editor is not playing. Its own level answers on the bound path, so joint "
                    "targets sent now would be written into the saved level's actor. Start PIE "
                    "and re-run.")
            live = play_copy(bound, play_world)
            if live is None:
                raise BridgeError(f"{JOINT_FUNCTION} is bound to {bound}, which is not in the "
                                  f"world being played ({play_world})")
            if not _readable(client, live, "Joints"):
                raise BridgeError(f"the play session has no {live}")
            return cls(client, live, "play session")
        if _read(client, *GAME_ENGINE) is not _ABSENT:
            return cls(client, bound, "game")
        # Neither engine answered: refuse rather than guess, because guessing "game" in an
        # editor would send through the preset into the saved level's actor.
        raise BridgeError("cannot tell whether this is the editor or a game (neither "
                          f"{EDITOR_ENGINE[0]} nor {GAME_ENGINE[0]} answered), so nothing is sent")

    # ── reading ──────────────────────────────────────────────────

    def read_joints(self):
        return self.client.get_setting("Joints", self.pose_path) or []

    def targets(self):
        return [float(j.get("TargetValue", 0.0)) for j in self.read_joints()]

    def applied(self, names):
        """Each named joint's CurrentValue -- what the skeleton is actually showing."""
        joints = self.read_joints()
        return {n: float(joints[self.index[n]].get("CurrentValue", 0.0)) for n in names}

    def _read_monitor(self):
        names = ("ArmJointNames", "StowedPose", "StowedTolerance", "WheelJointNames",
                 "WheelMotionThreshold", "WheelIdleTimeout")
        try:
            config = {name: self.client.get_setting(name, self.monitor_path) for name in names}
        except BridgeError:
            return None  # a display without a monitor: joints still move, indicators unchecked
        if len(config["ArmJointNames"] or []) != len(config["StowedPose"] or []):
            raise BridgeError(f"{self.monitor_path}: ArmJointNames and StowedPose differ in length")
        return config

    def arm_state(self):
        return self.client.get_setting("ArmPoseState", self.monitor_path)

    def wheels_moving(self):
        return bool(self.client.get_setting("bWheelsMoving", self.monitor_path))

    def status_text(self, widget):
        """The status bar's text for TXT_ArmStatus / TXT_ChairStatus, or None if not found."""
        if not self._status_root:
            # Probed again on every call until found: a failed probe is not remembered, so one
            # dropped request cannot turn every later indicator check into "not readable".
            roots = [(engine, n) for engine in STATUS_BAR_ENGINES for n in range(STATUS_BAR_INSTANCES)]
            try:
                found = self.client.read_many(
                    [(STATUS_BAR_PATH.format(engine=e, n=n, widget="TXT_ArmStatus"), "Text") for e, n in roots])
            except BridgeError:
                found = {}
            if not found:
                return None
            self._status_root = roots[max(found)]  # the newest play session's, if several answer
        engine, n = self._status_root
        path = STATUS_BAR_PATH.format(engine=engine, n=n, widget=widget)
        try:
            return plain_text(self.client.get_setting("Text", path))
        except BridgeError:
            return None

    # ── writing ──────────────────────────────────────────────────

    def send(self, targets):
        """Push a full joint vector and confirm it is what the display now holds.

        Full, because a short array is accepted and writes only the first joints -- on this
        display, the base's linkages. Confirmed, because nothing else proves the parameter name
        landed or that the call reached the live world rather than the editor's copy.
        """
        if len(targets) != len(self.joints):
            raise BridgeError(f"{len(targets)} values for {len(self.joints)} joints; a short "
                              "array would silently write only the first few")
        values = [float(v) for v in targets]
        if self.world == "play session":
            # Not through the preset. Its entry names the EDITOR's level, and Remote Control
            # resolves it there even during play: measured, a preset call wrote the editor's
            # copy -- the actor whose values Map_UI saves -- and left the running robot where it
            # was. So in PIE the game's own component is written directly.
            self.client.call_object(self.pose_path, JOINT_FUNCTION, Values=values)
        else:
            # A game (packaged or -game): the preset's path IS the live robot, and the preset is
            # the route an external sender uses, so exercise that.
            self.client.call(JOINT_FUNCTION, Values=values)
        landed = self.targets()
        wrong = [(self.joints[i].get("JointName"), sent, got)
                 for i, (sent, got) in enumerate(zip(targets, landed))
                 if within_limits(self.joints[i], sent) and abs(sent - got) > 1e-3]
        if len(landed) != len(targets) or wrong:
            raise BridgeError(
                f"the joint targets did not land on {self.pose_path}: e.g. {wrong[:3]}. Either "
                "the Values parameter arrived empty, or the preset entry is bound to a different "
                "object than the one being read (the editor's copy instead of the game's).")
        return landed

    def restore(self, original):
        """Put back the targets found at the start -- except the wheels, left where they stopped.

        Spinning them back would be a wheel-target change like any other: the chair would read
        DRIVING for a moment after the run had finished.
        """
        wheels = (self.monitor or {}).get("WheelJointNames") or []
        now = self.targets()
        self.send(with_joints(original, self.index,
                              {n: now[self.index[n]] for n in wheels if n in self.index}))

    def monitored(self, group):
        """The monitor's ArmJointNames or WheelJointNames, refused if the display lacks any of them."""
        if not self.monitor:
            raise BridgeError(f"no {MONITOR_COMPONENT} on {self.actor_path}, so nothing classifies the joints")
        names = list(self.monitor[group] or [])
        missing = [n for n in names if n not in self.index]
        if missing:
            raise BridgeError(f"{MONITOR_COMPONENT}.{group} names {missing}, which "
                              f"{self.pose_path} does not have")
        if not names:
            raise BridgeError(f"{MONITOR_COMPONENT}.{group} is empty")
        return names

    def poses(self):
        """(stowed, active) target vectors, wheels held where they are now."""
        arm, stowed = self.monitored("ArmJointNames"), self.monitor["StowedPose"]
        base = self.targets()
        stow = with_joints(base, self.index, dict(zip(arm, stowed)))
        nudge = active_offset(self.joints, self.index, arm, stowed, self.monitor["StowedTolerance"])
        return stow, with_joints(stow, self.index, nudge)


_ABSENT = object()


def _read(client, path, prop):
    """The property's value, or _ABSENT if the object or property does not answer."""
    try:
        return client.get_setting(prop, path)
    except BridgeError:
        return _ABSENT


def _readable(client, path, prop):
    return _read(client, path, prop) is not _ABSENT


def _poll_until(read, expected, timeout, sleep, clock):
    """Poll read() until it returns expected, or timeout. Returns the last value seen."""
    deadline = clock() + timeout
    value = read()
    while value != expected and clock() < deadline:
        sleep(0.05)
        value = read()
    return value


# Looked up at call time, not bound as defaults, so tests can run the timed checks on a fake
# clock instead of sleeping through them.
def _sleep(seconds):
    time.sleep(seconds)


def _clock():
    return time.monotonic()


def run_joint_checks(rig, check, sleep=None, clock=None, rate=10.0, step=5.0, frames=8):
    """Stow -> active -> stow, then drive -> park, each confirmed by readback.

    Three arm pushes rather than two because the monitor only announces a CHANGE: a stow push
    while already stowed fires nothing, so this forces two real transitions whatever state it
    starts in. Every push carries the wheels where they are and every drive frame carries the
    stowed arm, or one check would flip the other indicator. Restores the targets it found,
    except the wheels (see JointRig.restore).

    Do not run it while something else is streaming joints (the RAMMP UI bridge, the Gen1
    prototype): the other sender's next frame overwrites these, and the checks fail for it.
    """
    sleep = sleep or _sleep
    clock = clock or _clock
    try:
        wheels = rig.monitored("WheelJointNames")
        stow, active = rig.poses()
    except BridgeError as exc:
        check(False, "the display's joint-motion monitor watches joints it has", str(exc))
        return
    original = rig.targets()
    bar = rig.status_text("TXT_ArmStatus") is not None
    check(bar, "the HUD's status bar is readable",
          "no RAMMS_StatusBar under " + STATUS_BAR_PATH.format(engine="{UnrealEdEngine_0|GameEngine_0}",
                                                              n=f"{{0..{STATUS_BAR_INSTANCES - 1}}}",
                                                              widget="TXT_ArmStatus")
          + " -- the indicator checks below are skipped. Is the HUD up (not Simulate), with W_RAMMS_UI unrenamed?")

    def indicator(widget, expected, what):
        if not bar:
            return
        text = _poll_until(lambda: rig.status_text(widget), expected, 1.0, sleep, clock)
        check(text == expected, f"{what}: status bar reads {expected!r}",
              f"it reads {text!r}. A status bar built before rammp-org/ramms-ui's start-up fix shows "
              "its designer default until the first change")

    try:
        for name, pose, expected in (("arm stowed", stow, "Stowed"), ("arm active", active, "Active"),
                                     ("arm stowed again", stow, "Stowed")):
            try:
                rig.send(pose)
            except BridgeError as exc:
                check(False, f"{name}: targets landed", str(exc))
                return
            check(True, f"{name}: targets landed")
            # Landing proves the display RECEIVED the joints; CurrentValue proves the skeleton
            # moved to them. A pose component that eases toward its targets gets time to arrive.
            arm = rig.monitor["ArmJointNames"]
            want = {n: pose[rig.index[n]] for n in arm}

            def arrived():
                return all(abs(v - want[n]) <= 0.5 for n, v in rig.applied(arm).items())
            _poll_until(arrived, True, 3.0, sleep, clock)
            lag = {n: round(v - want[n], 2) for n, v in rig.applied(arm).items() if abs(v - want[n]) > 0.5}
            check(not lag, f"{name}: arm joints moved to their targets", f"still off by {lag}")
            state = _poll_until(rig.arm_state, expected, 1.0, sleep, clock)
            check(state == expected, f"{name}: monitor says {expected}", f"it says {state!r}")
            indicator("TXT_ArmStatus", expected, name)

        threshold = float(rig.monitor["WheelMotionThreshold"])
        idle = float(rig.monitor["WheelIdleTimeout"])
        if step <= threshold or 1.0 / rate >= idle:
            check(False, "drive stream outruns the monitor",
                  f"step {step} must exceed {threshold} and the frame period must be under {idle} s")
            return
        held = {name: stow[rig.index[name]] for name in wheels}
        moving_seen, driving_text = False, None
        for k in range(1, frames + 1):
            frame = with_joints(stow, rig.index, {n: v + k * step for n, v in held.items()})
            try:
                rig.send(frame)
            except BridgeError as exc:
                check(False, f"wheels turning: frame {k} landed", str(exc))
                return
            moving_seen = moving_seen or rig.wheels_moving()
            if bar and k >= 2 and driving_text != "DRIVING":
                # Read WHILE the frames are still coming: after they stop, the chair parks
                # within WheelIdleTimeout, and a slow read would see that instead.
                driving_text = rig.status_text("TXT_ChairStatus")
            sleep(1.0 / rate)
        check(moving_seen, "wheels turning: monitor says moving",
              f"{frames} frames of +{step} deg at {rate} Hz never registered")
        if bar:
            check(driving_text == "DRIVING", "wheels turning: status bar reads 'DRIVING'",
                  f"it read {driving_text!r} while the wheels were turning")
        sleep(idle)
        moving = _poll_until(rig.wheels_moving, False, idle + 1.0, sleep, clock)
        check(not moving, f"wheels stopped: monitor says parked within {idle} s + margin")
        indicator("TXT_ChairStatus", "PARKED", "wheels stopped")
    finally:
        try:
            rig.restore(original)
            check(True, "joint targets put back as found (the wheels stay where they stopped)")
        except BridgeError as exc:
            check(False, "joint targets put back as found", f"{exc} -- the robot is left in the test's pose")


# ── HUD readback: panel buttons and POIs ─────────────────────────────
#
# Everything the HUD draws for a push is a widget Remote Control can read: a panel button's
# PanelButton (id, label, style), its label text and opacity and its brush, and a POI's object and
# opacity. They live under the play session's game instance (measured in RammpUI PIE, 2026-11-20):
#   buttons  <GameInstance>.W_RAMMS_UI_C_0.WidgetTree_0.RAMMS_Panel.WidgetTree_0.RAMMS_Button_C_<k>
#   POIs     <GameInstance>.RAMMS_POI_C_<k>
#   layer    <GameInstance>.W_RAMMS_UI_C_0.WidgetTree_0.RammsDetectionPOILayer
# k only grows (UE never reuses an object name) and widgets a push replaced linger until garbage
# collection, so a widget is live only while it is still in its parent: a non-empty Slot.

PANEL_BUTTON_PATH = "{root}.W_RAMMS_UI_C_0.WidgetTree_0.RAMMS_Panel.WidgetTree_0.RAMMS_Button_C_{k}"
POI_PATH = "{root}.RAMMS_POI_C_{k}"
POI_LAYER_PATH = "{root}.W_RAMMS_UI_C_0.WidgetTree_0.RammsDetectionPOILayer"
HUD_PROBE_CHUNK = 1024
HUD_PROBE_LIMIT = 16384


def find_game_instance(client):
    """'/Engine/Transient.<engine>:GameInstance_<n>' of the running HUD, or None (one batch)."""
    roots = [f"/Engine/Transient.{e}:GameInstance_{n}"
             for e in STATUS_BAR_ENGINES for n in range(STATUS_BAR_INSTANCES)]
    # Not caught: a refused connection or an in-band error is a real problem, not "no HUD". Paths that
    # don't resolve come back as per-request 400s, which read_many leaves out.
    found = client.read_many([(STATUS_BAR_PATH.format(engine=e, n=n, widget="TXT_ArmStatus"), "Text")
                              for e in STATUS_BAR_ENGINES for n in range(STATUS_BAR_INSTANCES)])
    return roots[max(found)] if found else None


def probe_indexed(client, template, prop, chunk=HUD_PROBE_CHUNK, limit=HUD_PROBE_LIMIT, stop_after_empty=None):
    """{k: value of prop} for every '<template with k>' that answers, one /remote/batch per chunk.

    stop_after_empty=N stops after N empty chunks that follow a hit. That is safe only where the live
    widgets are the newest ones, contiguous at the top: panel buttons, rebuilt on every push. POIs are
    kept per object, so a long-lived one can sit far below newer ones; scan those to the limit. Past
    the limit the scan carries on while chunks still answer, so a long session is not cut off.
    """
    hits, empty_after_hit, start = {}, 0, 0
    while start < limit or (hits and max(hits) >= start - chunk):
        ks = range(start, start + chunk)
        found = client.read_many([(template.format(k=k), prop) for k in ks])
        start += chunk
        if found:
            hits.update({ks[i]: value for i, value in found.items()})
            empty_after_hit = 0
        elif hits and stop_after_empty is not None:
            empty_after_hit += 1
            if empty_after_hit >= stop_after_empty:
                break
        if start >= 1 << 20:
            break
    return hits


def unmangle(struct):
    """A Blueprint struct's fields without their GUID suffix: label_4_2D47...F -> label."""
    return {re.sub(r"_\d+_[0-9A-F]{32}$", "", key): value for key, value in (struct or {}).items()}


def rich_tag(rich):
    """'btn-light' from '<btn-light>Label</>'; None for plain text."""
    match = re.match(r"\s*<([^/>]+)>", rich or "")
    return match.group(1) if match else None


def _tint(widget_style):
    tint = ((widget_style or {}).get("Normal") or {}).get("TintColor") or {}
    color = tint.get("SpecifiedColor") or {}
    return tuple(round(float(color.get(c, 0.0)), 3) for c in "RGBA") if color else None


def read_panel_buttons(client, root):
    """The panel's live buttons, in creation order: id, label, style as sent, and what is drawn."""
    template = PANEL_BUTTON_PATH.replace("{root}", root)
    # Every push builds new buttons, so the live ones are the newest: stopping early is safe here.
    panel_buttons = probe_indexed(client, template, "PanelButton", stop_after_empty=2)
    ks = sorted(panel_buttons)
    slots = client.read_many([(template.format(k=k), "Slot") for k in ks]) if ks else {}
    live = [k for i, k in enumerate(ks) if slots.get(i)]
    reads = []
    for k in live:
        path = template.format(k=k)
        reads += [(f"{path}.WidgetTree_0.TXT_Button", "Text"), (f"{path}.WidgetTree_0.TXT_Button", "RenderOpacity"),
                  (f"{path}.WidgetTree_0.BTN", "WidgetStyle")]
    values = client.read_many(reads) if reads else {}
    rows = []
    for j, k in enumerate(live):
        sent = unmangle(panel_buttons[k])
        text = values.get(3 * j)
        opacity = values.get(3 * j + 1)
        rows.append({
            "k": k, "button_id": sent.get("button_id", ""), "label": sent.get("label", ""),
            "style": sent.get("style", ""), "text": plain_text(text) if text is not None else None,
            "tag": rich_tag(text), "opacity": round(float(opacity), 3) if opacity is not None else None,
            "tint": _tint(values.get(3 * j + 2)),
        })
    return rows


def button_look(row):
    """What a button looks like, as far as Remote Control can tell: brush tint, label opacity, text style."""
    return (row["tint"], row["opacity"], row["tag"])


def check_button_styles(pushed, rows):
    """[(status, what, detail)] for a pushed panel against its live buttons; status pass | FAIL | warn.

    Asserts what any intent for the styles needs: one live button per pushed one, each showing its own
    label, unknown and differently-cased names drawn like the style they fall back to, and the UI's
    styles told apart. A contract style drawn like another is a warning, not a failure: the UI may
    have moved on from the contract on purpose, but whoever sends it should know.
    """
    results = []
    by_id = {}
    for row in rows:
        by_id.setdefault(row["button_id"], []).append(row)
    looks = {}
    for b in pushed:
        found = by_id.get(b["button_id"], [])
        what = f"{b['button_id']} is on the panel once"
        if len(found) != 1:
            results.append(("FAIL", what, f"found {len(found)} live buttons with that id"))
            continue
        results.append(("pass", what, ""))
        row = found[0]
        shown = row["text"]
        results.append(("pass" if shown == b["label"] else "FAIL", f"{b['button_id']} shows its label {b['label']!r}",
                        f"it shows {shown!r}"))
        looks[b["button_id"]] = (b.get("style"), button_look(row))

    reference = {}
    for style, look in looks.values():
        if style and style in UI_BUTTON_STYLES:
            reference.setdefault(style, look)
    for button_id, (style, look) in looks.items():
        target = ui_button_style(style)
        # Only names that are not themselves a UI style; warning is reported against the contract below.
        if style in UI_BUTTON_STYLES or style == "warning" or target not in reference:
            continue
        results.append(("pass" if look == reference[target] else "FAIL",
                        f"{button_id} (style {style!r}) looks like {target}",
                        f"{look} vs {target}'s {reference[target]}"))
    names = [s for s in UI_BUTTON_STYLES if s in reference]
    for i, a in enumerate(names):
        for b2 in names[i + 1:]:
            results.append(("pass" if reference[a] != reference[b2] else "FAIL", f"{a} and {b2} look different",
                            f"both look like {reference[a]}"))
    contract = {s: look for s, look in looks.values() if s in CONTRACT_BUTTON_STYLES}
    if "warning" in contract and "normal" in contract and contract["warning"] == contract["normal"]:
        results.append(("warn", "the contract's warning looks like normal",
                        "the Orchestrator sends its Cancel buttons as \"warning\"; RAMMS_Button draws "
                        "them like any other button"))
    return results


def read_pois(client, root, ks=None):
    """The POI layer's live POIs: the object each shows, and its opacity (ColorAndOpacity alpha).

    ks limits the read to POIs already found (re-reading opacity after a tap creates no new POIs);
    without it every candidate name is scanned.
    """
    template = POI_PATH.replace("{root}", root)
    if ks is None:
        objects = probe_indexed(client, template, "DetectedObjectData")
    else:
        values = client.read_many([(template.format(k=k), "DetectedObjectData") for k in ks])
        objects = {ks[i]: value for i, value in values.items()}
    ks = sorted(objects)
    reads = [(template.format(k=k), prop) for k in ks for prop in ("Slot", "ColorAndOpacity")]
    values = client.read_many(reads) if reads else {}
    rows = []
    for j, k in enumerate(ks):
        if not values.get(2 * j):
            continue
        color = values.get(2 * j + 1) or {}
        data = objects[k] or {}
        rows.append({"k": k, "instance_id": data.get("InstanceId", ""), "class": data.get("PerceptionClass", ""),
                     "alpha": round(float(color.get("A", 1.0)), 3)})
    return rows


def read_poi_layer(client, root):
    """The layer's dim opacity and visibility (None where Remote Control won't say)."""
    path = POI_LAYER_PATH.replace("{root}", root)
    values = client.read_many([(path, "UnselectedPOIOpacity"), (path, "Visibility")])
    opacity = values.get(0)
    return {"unselected_opacity": float(opacity) if opacity is not None else None, "visibility": values.get(1)}


def check_pois(pushed_objects, rows):
    """[(status, what, detail)]: one live POI per pushed object with an id, as many id-less ones as pushed."""
    results = []
    ids = [o.get("instance_id", "") for o in pushed_objects]
    live_ids = [r["instance_id"] for r in rows]
    for instance_id in [i for i in ids if i]:
        n = live_ids.count(instance_id)
        results.append(("pass" if n == 1 else "FAIL", f"one POI for {instance_id}", f"found {n}"))
    unkeyed = ids.count("")
    n = live_ids.count("")
    results.append(("pass" if n == unkeyed else "FAIL", f"{unkeyed} POI(s) without an id", f"found {n}"))
    extra = sorted({i for i in live_ids if i and i not in ids})
    results.append(("pass" if not extra else "FAIL", "no POIs left over from an earlier push", f"still showing {extra}"))
    return results


def check_poi_dimming(rows, tapped_id, dim, tolerance=1e-3):
    """[(status, what, detail)]: with tapped_id last tapped, it is opaque and every other POI is at dim.

    tapped_id None means nothing is tapped (or the tapped object is gone): every POI opaque.
    """
    if dim is None:
        return [("FAIL", "the POI layer's UnselectedPOIOpacity is readable",
                 "no such property: this RammsUI predates POI dimming, or the layer has another name or path")]
    results = []
    for row in rows:
        name = row["instance_id"] or f"(no id, POI {row['k']})"
        want = 1.0 if tapped_id is None or row["instance_id"] == tapped_id else dim
        results.append(("pass" if abs(row["alpha"] - want) <= tolerance else "FAIL",
                        f"POI {name} at opacity {want:g}", f"it is at {row['alpha']:g}"))
    return results


def tap_origin(event):
    """Which surface an object_tapped came from, by what it carries.

    - 'camera image': a tap_point, or a camera stream id ('camera/scene/color', 'engine/overlay') --
      a marker tap without a point (no frame yet) still names its camera's stream;
    - '3D view': a bare source alias ('scene', 'overlay'), which world markers send;
    - 'POI layer': neither. A world marker whose object had no source sends neither too, so this
      one is a best guess.
    """
    stream_id = event.get("stream_id") or ""
    if "tap_point" in event or "/" in stream_id:
        return "camera image"
    return "3D view" if stream_id else "POI layer"


def report(results, check):
    """Feed check_* results to a selftest-style check(); a warn only prints."""
    for status, what, detail in results:
        if status == "warn":
            print(f"  warn  {what} -- {detail}")
        else:
            check(status == "pass", what, detail)


def cmd_discover(client, args):
    preset = client.load_preset()
    print(f"preset: {preset.get('Name', client.preset)}")
    print("\nexposed functions (call by LABEL, not by C++ name):")
    for name, label in sorted(client._labels.items()):
        argnames = client._arguments.get(name) or []
        owners = client._owners.get(name)
        if owners is None:
            bound = "(not reported by this preset)"
        elif owners:
            bound = ", ".join(owners)
        else:
            bound = "!! BOUND TO NOTHING !!"
        note = (f"  (RC omits it; the argument is {KNOWN_ARGUMENTS[name]})"
                if not argnames and name in KNOWN_ARGUMENTS else "")
        print(f"  {name:<24} label={label!r:<28} args={argnames}{note}")
        print(f"  {'':<24} bound to: {bound}")
        labels = client._duplicates.get(name) or []
        if len(labels) > 1:
            why = ("it is the bound one" if owners
                   else "none of them are bound; using the last")
            print(f"  {'':<24} NOTE: exposed {len(labels)} times as {labels}; "
                  f"using {label!r} -- {why}")
    for group in preset.get("Groups", []):
        for prop in group.get("ExposedProperties", []):
            print(f"  property: {prop.get('DisplayName')!r}")

    print("\nlive camera stream settings:")
    for prop in ("SceneColorStreamID", "SceneDataStreams", "SceneDetectionSources",
                 "OverlayColorStreamID", "OverlayDataStreams", "OverlayDetectionSources"):
        try:
            print(f"  {prop:<24} {json.dumps(client.get_setting(prop))}")
        except BridgeError as exc:
            print(f"  {prop:<24} <unreadable: {exc}>")

    unbound = client.unbound_functions()
    if unbound:  # None means "could not tell", which load_preset already raised on
        print(f"\nPROBLEM: {len(unbound)} exposed function(s) are bound to no object:")
        for name in unbound:
            print(f"  {name}")
        print("  Remote Control answers HTTP 200 for these and the call reaches")
        print("  nothing, so pushes look like they worked and the UI never changes.")
        print("  It happens when the object an entry is bound to is moved or")
        print("  replaced -- the level holding RammsBridgeReceiver, or the robot")
        print("  display actor -- and the preset still points at the old path.")
        for name in unbound:
            print(f"    {name}: rebind to {REBIND_TARGETS.get(name, 'the RammsBridgeReceiver')}")
        return 1

    latest, _ = client.poll_events(0)
    print(f"\nUI event sequence is at {latest}")
    return 0


def cmd_panel(client, args):
    if args.advance or args.phases or args.pct is not None:
        if args.scenario not in ("progress", "paused"):
            print("--pct, --advance and --phases only apply to the progress panels", file=sys.stderr)
            return 2
        if sum(bool(x) for x in (args.advance, args.phases, args.scales)) > 1:
            print("--advance, --phases and --scales are separate runs; pick one", file=sys.stderr)
            return 2

        if args.scales:
            for fraction, percent in PROGRESS_SCALE_PAIRS:
                for pct in (fraction, percent):
                    client.update_panel(progress_panel(pct))
                    print(f"pushed pct={pct:g}")
                    time.sleep(args.interval)
            print("  expect: the bar did not move between the two values of each pair --")
            print("          0.25 then 25, 0.65 then 65, 1.0 then 100 are the same bar.")
            return 0

        if args.phases:
            pushes = [(p, p["progress"]["pct"], p["message"]) for p in PROGRESS_PANELS]
            what = ("each panel showed ITS OWN fraction, not the previous one's, and the\n"
                    "          title, message and buttons changed with it")
        else:
            steps = PROGRESS_STEPS if args.advance else (args.pct,)
            pushes = [(progress_panel(pct), pct, None) for pct in steps]
            what = ("the bar moved through each value while the title, message and buttons\n"
                    "          stayed put -- it should update in place, not rebuild the panel")

        for i, (pan, pct, label) in enumerate(pushes):
            client.update_panel(pan)
            print(f"pushed pct={pct:.2f}" + (f"  ({label})" if label else ""))
            if i + 1 < len(pushes):
                time.sleep(args.interval)
        print(f"  expect: {what}")
        return 0

    pan, expectation = PANEL_SCENARIOS[args.scenario]
    ids = scenario_buttons(args.scenario)
    # Before the push, so a fast tap is not left behind the baseline.
    since = _current_sequence(client) if (args.wait and ids) else None
    client.update_panel(pan)
    print(f"pushed panel scenario {args.scenario!r}")
    print(f"  expect: {expectation}")
    print(f"  button_ids: {ids or '(none)'}")
    if args.wait and ids:
        return _await_button(client, ids, args.timeout, since=since)
    return 0


def cmd_objects(client, args):
    if args.clear and (args.count is not None or args.set != "a"):
        print("--clear pushes an empty list, so --set and --count do nothing", file=sys.stderr)
        return 2
    if args.pixels and args.set != "a":
        print("--pixels only has a pixel-space set for --set a", file=sys.stderr)
        return 2
    # None means "discover it"; an explicit --stream-id '' is honoured as empty, which
    # is what the real bridge sends and is not the same request at all.
    stream_id = args.stream_id
    if stream_id is None:
        prop = "SceneColorStreamID" if args.source == "scene" else "OverlayColorStreamID"
        try:
            stream_id = client.get_setting(prop) or ""
        except BridgeError:
            stream_id = ""
    objects = [] if args.clear else sample_objects(
        args.source, args.pixels, args.set, args.count)
    payload = detected_objects_payload(objects, args.select or "", stream_id)
    since = _current_sequence(client) if args.wait else None
    client.update_detected_objects(payload)
    if args.clear:
        print(f"cleared detected objects on stream_id={stream_id!r}")
        print("  expect: every box AND every POI disappears from the view -- an empty")
        print("          list is the full set, so it clears every stream, whatever")
        print("          stream_id it carries.")
        return 0
    space = "pixel (1280x720)" if args.pixels else "normalized 0-1"
    print(f"pushed {len(objects)} objects on stream_id={stream_id!r} source={args.source!r} "
          f"in {space} coordinates")
    print(f"  expect: boxes for {[o['instance_id'] for o in objects]}"
          + (f", with {args.select!r} highlighted" if args.select else ""))
    if args.pixels:
        print("  note: pixel boxes are divided by the live frame size, or by "
              "FallbackDetectionImageSize (640x480) if nothing is streaming -- so "
              "these only land correctly against a 1280x720 feed.")
    if args.wait:
        wanted = [o["instance_id"] for o in objects if o["instance_id"]]
        if not wanted:
            print("  note: none of these objects has an instance_id, and a tap on one sends no id,")
            print("        so there is nothing to wait for. Skipping the wait.")
            return 0
        return _await_tap(client, wanted, args.timeout, since=since)
    return 0


def cmd_worldstate(client, args):
    payload = {
        "objects": [
            {"instance_id": "cup_1", "perception_class": "cup",
             "state": "full", "location": "counter"},
            {"instance_id": "fridge_1", "perception_class": "fridge",
             "state": "closed", "location": "kitchen"},
        ],
        "selected_instance_id": "cup_1",
    }
    client.update_world_state(payload)
    print("pushed world state (2 objects, cup_1 selected)")
    print("  expect: whatever widget consumes world state updates; no camera change.")
    return 0


def cmd_camera(client, args):
    if args.state == "open":
        client.open_camera_feed()
        print("requested camera feed OPEN -- expect the feed to appear.")
    else:
        client.close_camera_feed()
        print("requested camera feed CLOSE -- expect the feed to disappear.")
    print("  note: the UI exposes no readable feed state, so this is eyes-only.")
    return 0


def cmd_events(client, args):
    since = args.since
    if args.since is None:
        since, _ = client.poll_events(0)
        print(f"watching from current sequence {since} (use --since 0 for history)")
    deadline = time.monotonic() + args.timeout if args.timeout else None
    seen_expected = False
    try:
        while True:
            latest, events = client.poll_events(since)
            for event in events:
                since = max(since, int(event.get("seq", since)))
                print(f"  [{event.get('seq')}] {_describe(event)}")
                if args.expect and _event_id(event) == args.expect:
                    seen_expected = True
            if args.expect and seen_expected:
                print(f"OK: saw {args.expect!r}")
                return 0
            if not args.follow and not args.expect:
                return 0
            if deadline and time.monotonic() > deadline:
                if args.expect:
                    print(f"FAIL: {args.expect!r} not seen within {args.timeout:.0f}s")
                    return 1
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")
        if args.expect and not seen_expected:
            # --expect promises "exit 0 only once this id arrives". Cancelling
            # the wait leaves the assertion unmet, so a script must not read
            # it as a successful round-trip.
            print(f"FAIL: interrupted before {args.expect!r} arrived")
            return 1
        return 0


def _reporter():
    """A selftest-style check(): prints pass/FAIL and remembers failures."""
    failures = []

    def check(ok, what, detail=""):
        print(f"  {'pass' if ok else 'FAIL'}  {what}{(' -- ' + detail) if detail and not ok else ''}")
        if not ok:
            failures.append(what)
    return check, failures


def _describe_joints(rig):
    targets = rig.targets()
    print(f"joints: {JOINT_FUNCTION} -> {rig.pose_path}  ({rig.world})")
    if rig.world == "play session":
        print("  sent straight to the play session's component: during PIE the preset entry "
              "resolves to the editor's level, not the game")
    print(f"  {len(rig.joints)} joints, set by position; vectors are built by name from this list")
    if not rig.monitor:
        print(f"  no {MONITOR_COMPONENT} on {rig.actor_path}: joints move, but nothing classifies them")
        return
    m = rig.monitor
    for label, group in (("arm", m["ArmJointNames"]), ("wheels", m["WheelJointNames"])):
        where = ", ".join(f"{n}@{rig.index[n]}={targets[rig.index[n]]:.1f}" if n in rig.index
                          else f"{n}@MISSING" for n in group)
        print(f"  {label:<7} {where}")
    print(f"  stow pose {[round(v, 1) for v in m['StowedPose']]} +/- {m['StowedTolerance']} deg; "
          f"wheels move past {m['WheelMotionThreshold']} deg, park after {m['WheelIdleTimeout']} s")
    print(f"monitor: arm {rig.arm_state()}, wheels {'moving' if rig.wheels_moving() else 'still'}")
    arm, chair = rig.status_text("TXT_ArmStatus"), rig.status_text("TXT_ChairStatus")
    print(f"status bar: arm {arm!r}, chair {chair!r}" if arm is not None
          else "status bar: NOT FOUND under any game instance -- is the HUD up?")
    missing = [n for n in m["ArmJointNames"] + m["WheelJointNames"] if n not in rig.index]
    if missing:
        print(f"PROBLEM: the monitor watches {missing}, which this pose component does not have")


def cmd_joints(client, args):
    """Drive the robot display's arm and wheels, and check the status indicators follow."""
    try:
        rig = JointRig.locate(client)
    except BridgeError as exc:
        print(f"joints: {exc}", file=sys.stderr)
        return 1

    if args.check:
        check, failures = _reporter()
        run_joint_checks(rig, check, rate=args.rate, step=args.step,
                         frames=max(3, int(args.seconds * args.rate)))
        print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all joint checks passed'}")
        return 1 if failures else 0

    if args.set:
        values = {}
        for pair in args.set:
            name, sep, value = pair.partition("=")
            if not sep:
                raise BridgeError(f"--set takes NAME=DEGREES, got {pair!r}")
            values[name] = float(value)
        rig.send(with_joints(rig.targets(), rig.index, values))
        print(f"  sent {values}; everything else held")
    elif args.stow or args.activate:
        stow, active = rig.poses()
        rig.send(stow if args.stow else active)
        print(f"  sent the {'stowed' if args.stow else 'active'} arm pose, wheels held")
    elif args.drive:
        wheels = rig.monitored("WheelJointNames")
        base = rig.targets()  # the arm stays wherever it is
        held = {n: base[rig.index[n]] for n in wheels}
        frames = max(3, int(args.seconds * args.rate))
        for k in range(1, frames + 1):
            rig.send(with_joints(base, rig.index, {n: v + k * args.step for n, v in held.items()}))
            _sleep(1.0 / args.rate)
        print(f"  streamed {frames} frames of +{args.step} deg at {args.rate} Hz; "
              f"moving now: {rig.wheels_moving()} (expect the chair to read PARKED within "
              f"{rig.monitor['WheelIdleTimeout']} s)")
    elif args.sweep:
        stow, active = rig.poses()
        steps = max(4, int(args.seconds * args.rate))
        for k in range(steps + 1):
            # Out and back: the arm swings to the active pose and returns to stowed.
            t = 1.0 - abs(2.0 * k / steps - 1.0)
            rig.send([a + (b - a) * t for a, b in zip(stow, active)])
            _sleep(args.seconds / steps)
        print("  swept stowed -> active -> stowed")
    _describe_joints(rig)
    return 0


def _hud_root_or_explain(client):
    root = find_game_instance(client)
    if root is None:
        print("no HUD found: start PIE (or the game) with W_RAMMS_UI on screen. Remote Control can only "
              "read widgets in a running game", file=sys.stderr)
    return root


def _print_buttons(rows):
    for r in rows:
        tint = "#" + "".join(f"{int(round(v * 255)):02X}" for v in r["tint"][:3]) if r["tint"] else "?"
        print(f"  {r['button_id']:<22} style={r['style']!r:<14} shows={r['text']!r:<16} "
              f"tag={r['tag']} opacity={r['opacity']} tint={tint}")


def _await_buttons(client, root, pushed, timeout=3.0, sleep=None, clock=None):
    """The live buttons once every pushed id is on the panel (the panel builds them a tick or two later)."""
    sleep, clock = sleep or _sleep, clock or _clock
    want = {b["button_id"] for b in pushed}
    deadline = clock() + timeout
    rows = read_panel_buttons(client, root)
    while not want <= {r["button_id"] for r in rows} and clock() < deadline:
        sleep(0.2)
        rows = read_panel_buttons(client, root)
    return rows


def run_style_checks(client, root, scenario, check, sleep=None, clock=None):
    """Push a style scenario and check the live buttons against it."""
    pan = PANEL_SCENARIOS[scenario][0]
    client.update_panel(pan)
    rows = _await_buttons(client, root, pan["buttons"], sleep=sleep, clock=clock)
    _print_buttons(rows)
    report(check_button_styles(pan["buttons"], rows), check)
    return rows


def cmd_styles(client, args):
    """Push a style panel, read back what each button draws, and check it."""
    root = _hud_root_or_explain(client)
    if root is None:
        return 1
    check, failures = _reporter()
    print(f"panel scenario {args.scenario!r} on {root}")
    run_style_checks(client, root, args.scenario, check)
    if args.wait:
        ids = scenario_buttons(args.scenario)
        print(f"  tap any of {ids}: every style still reports its press today, unavailable included")
        _await_button(client, ids, args.timeout, allow_skip=True)
    print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all style checks passed'}")
    return 1 if failures else 0


def run_poi_checks(client, root, check, objects=None):
    """Push the sample objects to the POI layer and check one POI per object, opacity untouched."""
    objects = objects if objects is not None else sample_objects("scene")
    client.update_detected_objects(detected_objects_payload(objects, "", _scene_stream_id(client)))
    _sleep(0.5)
    layer = read_poi_layer(client, root)
    rows = read_pois(client, root)
    for r in rows:
        print(f"  POI {r['k']:<4} {r['instance_id'] or '(no id)':<10} {r['class']:<10} opacity={r['alpha']:g}")
    if layer["visibility"] == "Collapsed":
        print("  note: the POI layer is collapsed -- the scene camera is showing. Press the camera toggle on "
              "the status bar to see (and tap) the POIs")
    report(check_pois(objects, rows), check)
    dim = layer["unselected_opacity"]
    lit = [r for r in rows if r["alpha"] > (dim or 0) + 1e-3]
    if dim is not None and rows and len(lit) == 1 and len(rows) > 1:
        print(f"  note: {lit[0]['instance_id']!r} is still the last tapped POI from earlier, so the others "
              "are dimmed. Tap empty background to clear it")
    else:
        report(check_poi_dimming(rows, None, dim or 0.0), check)
    return rows, layer


SKIPPED = "skipped"


def _can_poll_stdin():
    """Whether Enter can be noticed without blocking: a terminal, on POSIX or a Windows console.

    Piped or captured stdin can't: a wait there runs to its timeout, and a step with
    no timeout falls back to input().
    """
    try:
        if sys.stdin is None or not sys.stdin.isatty():
            return False
    except (AttributeError, OSError, ValueError):
        return False
    return msvcrt is not None if os.name == "nt" else True


def _enter_pressed():
    """Whether Enter was pressed (without blocking); False where stdin can't be polled."""
    if not _can_poll_stdin():
        return False
    try:
        if os.name == "nt":
            while msvcrt.kbhit():
                if msvcrt.getwch() in "\r\n":
                    return True
            return False
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return True
    except (OSError, ValueError):
        pass
    return False


def _next_event(client, wanted, kind, timeout, since, sleep=None, clock=None):
    """The first `kind` event after `since` carrying one of `wanted`; None at the timeout, SKIPPED on Enter."""
    sleep, clock = sleep or _sleep, clock or _clock
    deadline = clock() + timeout
    while clock() < deadline:
        if _enter_pressed():
            return SKIPPED
        _, events = client.poll_events(since)
        for event in events:
            since = max(since, int(event.get("seq", since)))
            print(f"    [{event.get('seq')}] {_describe(event)}")
            if event.get("event_type") == kind and _event_id(event) in wanted:
                return event
        sleep(0.4)
    return None


def _poll_dimming(client, root, ks, tapped_id, dim, timeout, sleep=None, clock=None):
    """check_poi_dimming once it passes, or as it stands at the timeout; SKIPPED on Enter."""
    sleep, clock = sleep or _sleep, clock or _clock
    deadline = clock() + timeout
    while True:
        results = check_poi_dimming(read_pois(client, root, ks), tapped_id, dim)
        if all(status == "pass" for status, _, _ in results) or clock() >= deadline:
            return results
        if _enter_pressed():
            return SKIPPED
        sleep(0.2)


def _tap_round_trip(client, root, rows, dim, timeout, check):
    """Ask for a tap on a POI, check the others dim; ask for a tap on the background, check they return."""
    ks = [r["k"] for r in rows]
    ids = [r["instance_id"] for r in rows if r["instance_id"]]
    print(f"\n  tap one of the POIs {ids} (it dims the others to {dim:g}); Enter skips")
    since = _current_sequence(client)
    event = _next_event(client, set(ids), "object_tapped", timeout, since)
    if event is SKIPPED:
        print("  skipped")
        return
    if event is None:
        check(False, "a POI was tapped", f"no object_tapped for {ids} within {timeout:g} s")
        return
    tapped = event.get("instance_id")
    check(tap_origin(event) == "POI layer", f"the tap on {tapped} came from the POI layer",
          f"it came from the {tap_origin(event)}: tap a POI, not a box on a camera image")
    results = _poll_dimming(client, root, ks, tapped, dim, 2.0)
    if results is SKIPPED:
        print("  skipped")
        return
    report(results, check)
    print("\n  now tap empty background: that clears the selection and every POI goes opaque again; Enter skips")
    results = _poll_dimming(client, root, ks, None, dim, timeout)
    if results is SKIPPED:
        print("  skipped")
        return
    report(results, check)


def cmd_pois(client, args):
    """Push objects to the POI layer; check them, and with --tap, that a tap dims the rest."""
    root = _hud_root_or_explain(client)
    if root is None:
        return 1
    check, failures = _reporter()
    rows, layer = run_poi_checks(client, root, check)
    if args.tap and rows:
        dim = layer["unselected_opacity"]
        if dim is None:
            report(check_poi_dimming(rows, None, None), check)
        else:
            _tap_round_trip(client, root, rows, dim, args.timeout, check)
    print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all POI checks passed'}")
    return 1 if failures else 0


# ── world markers derived from 2D: the depth fallback ────────────────
#
# URammsDetectionWorldMarkers places an object with no position_m by unprojecting its centroid
# through the camera named by its `source`: that stream's intrinsics, the range read from its depth
# sibling (same group), and the stream's pose. `markers` checks it against a camera whose answer is
# known: it streams one itself, with its intrinsics and pose in the RMSS metadata the stream bridge
# already reads, and works out independently where each centroid must land.
#
# The component is driven through the editor's Python, not Remote Control: the project's Remote
# Control allow-list covers only the bridge receiver (rightly), and the component is not something a
# robot UI calls. That makes this an editor-only check, in a play session.

# Its own stream ids: the stream bridge reads a stream's intrinsics and size only when it first sees
# it, so a scene stream first sent without intrinsics -- the synthetic video `walkthrough` and
# `stream` send, or an older bridge -- never gains them, however many later frames carry them.
# Asymmetric on purpose (fx != fy, the principal point off-centre, a roll in the pose), so a swapped
# axis or a dropped angle cannot pass by symmetry. Fixed for the same reason as the ids: a second
# run with other values would be read against the first run's registration.
MARKER_CAMERA = {
    "color_id": "test/markers/color",
    "depth_id": "test/markers/depth",
    "group": "test/markers",
    "channels": (5, 6),
    "size": (320, 240),
    "intrinsics": {"fx": 260.0, "fy": 240.0, "cx": 158.0, "cy": 122.0},
    "transform": {"x": 100.0, "y": -50.0, "z": 120.0, "pitch": -10.0, "yaw": 30.0, "roll": 5.0},
    # Which frames carry the intrinsics and pose. The UI bridge puts them on the DEPTH frames only
    # (ui_bridge_node.py _forward_scene_camera_depth); "both" is another producer's way.
    "meta_on": "depth",
}
# The same camera with its intrinsics and pose on the colour frames too -- its own ids, because a
# stream's registration keeps intrinsics from earlier frames, so one mode would mask the other.
MARKER_CAMERA_COLOUR_META = dict(MARKER_CAMERA, color_id="test/markers-colour-meta/color",
                                 depth_id="test/markers-colour-meta/depth", group="test/markers-colour-meta",
                                 channels=(7, 8), meta_on="both")
MARKER_WALL_MM = 2000                       # the back wall, everywhere not covered below
MARKER_BOX_MM = 1200                        # a box in front of it
MARKER_BOX = (0.40, 0.40, 0.60, 0.60)       # its normalized rectangle
MARKER_DROPOUT = (0.15, 0.15, 0.30, 0.30)   # no return at all (zeros), as a shiny or dark surface gives
MARKER_TOLERANCE_CM = 0.5
MARKER_PREFIX = "rmtest_"                   # every test object's id and class: how its markers are told apart


def marker_depth_image(camera=MARKER_CAMERA, box_mm=MARKER_BOX_MM):
    """The test camera's depth, uint16 millimetres: a wall, a box in front of it, a dropout patch."""
    import numpy as np
    w, h = camera["size"]
    depth = np.full((h, w), MARKER_WALL_MM, np.uint16)
    for (x0, y0, x1, y1), value in ((MARKER_BOX, box_mm), (MARKER_DROPOUT, 0)):
        depth[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)] = value
    return depth


def sample_depth_cm(depth, nx, ny, radius):
    """The range at a normalized point, in cm, as the component reads it; None with no return.

    The median of the non-zero samples in a (2r+1)^2 window around the pixel the point falls in,
    clamped to the image -- SampleDepthUU in RammsDetectionWorldMarkers.cpp.
    """
    h, w = depth.shape
    cx = min(max(int(math.floor(nx * w)), 0), w - 1)
    cy = min(max(int(math.floor(ny * h)), 0), h - 1)
    r = min(max(int(radius), 0), 8)
    window = depth[max(0, cy - r):min(h, cy + r + 1), max(0, cx - r):min(w, cx + r + 1)]
    samples = sorted(float(v) * 0.1 for v in window.ravel() if v > 0)
    return samples[len(samples) // 2] if samples else None


def ue_rotate(pitch, yaw, roll, v):
    """v rotated by FRotator(pitch, yaw, roll), degrees -- FRotationMatrix's axes, row-vector form."""
    sp, cp = math.sin(math.radians(pitch)), math.cos(math.radians(pitch))
    sy, cy = math.sin(math.radians(yaw)), math.cos(math.radians(yaw))
    sr, cr = math.sin(math.radians(roll)), math.cos(math.radians(roll))
    x_axis = (cp * cy, cp * sy, sp)
    y_axis = (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp)
    z_axis = (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp)
    return tuple(v[0] * x_axis[i] + v[1] * y_axis[i] + v[2] * z_axis[i] for i in range(3))


def camera_local_to_world(local, transform):
    """A point in the camera's Unreal-local frame (X along the view, Y right, Z up), in cm, to world."""
    r = ue_rotate(transform["pitch"], transform["yaw"], transform["roll"], local)
    return (r[0] + transform["x"], r[1] + transform["y"], r[2] + transform["z"])


def expected_derived(nx, ny, depth_cm, camera=MARKER_CAMERA):
    """Where a normalized centroid at depth_cm belongs: pinhole unprojection, then the stream's pose.

    u = nx * width; X (the view axis) = depth; Y = (u - cx) * depth / fx; Z = -(v - cy) * depth / fy
    -- RammsProjection.ush's mapping inverted, which the projector and the marker share.
    """
    w, h = camera["size"]
    k = camera["intrinsics"]
    u, v = nx * w, ny * h
    local = (depth_cm, (u - k["cx"]) * depth_cm / k["fx"], -(v - k["cy"]) * depth_cm / k["fy"])
    return camera_local_to_world(local, camera["transform"])


def expected_sent_camera(position_m, camera=MARKER_CAMERA):
    """A position_m in the optical frame (X right, Y down, Z forward; metres), placed through the pose."""
    x, y, z = position_m
    return camera_local_to_world((z * 100.0, x * 100.0, -y * 100.0), camera["transform"])


def expected_sent_world(position_m):
    """A position_m in the producer's world frame (right-handed, Z up; metres): Y is negated."""
    x, y, z = position_m
    return (x * 100.0, -y * 100.0, z * 100.0)


class MarkerCameraFeeder:
    """Streams MARKER_CAMERA -- colour and depth, each with its intrinsics and pose -- until stopped."""

    def __init__(self, host, port, camera=MARKER_CAMERA, rate=10.0):
        self.host, self.port, self.camera, self.rate = host, port, camera, rate
        self.sender = None
        self.error = None
        self._stop = threading.Event()
        self._thread = None
        self.box_mm = MARKER_BOX_MM
        self._frames = None

    def set_box_mm(self, box_mm):
        """Move the box: the next frames carry it at this depth (a fresh value tells fresh depth from stale)."""
        self.box_mm = box_mm
        self._frames = None

    def metadata(self):
        """The RMSS metadata RammsStreamCameraBridge reads: intrinsics {fx,fy,cx,cy}, transform {x..roll}."""
        return {"intrinsics": dict(self.camera["intrinsics"]), "transform": dict(self.camera["transform"])}

    def frames(self):
        import numpy as np
        w, h = self.camera["size"]
        colour = np.zeros((h, w, 4), np.uint8)
        colour[..., 0] = np.linspace(40, 220, w, dtype=np.uint8)[None, :]
        colour[..., 1] = np.linspace(40, 220, h, dtype=np.uint8)[:, None]
        colour[..., 3] = 255
        return colour.tobytes(), marker_depth_image(self.camera, self.box_mm).tobytes()

    def send_once(self, colour, depth):
        w, h = self.camera["size"]
        meta = self.metadata()
        colour_ch, depth_ch = self.camera["channels"]
        colour_meta = meta if self.camera.get("meta_on", "depth") == "both" else None
        self.sender.send_image(colour_ch, colour, w, h, fmt="bgra8", metadata=colour_meta, group=self.camera["group"],
                               role="color", stream_id=self.camera["color_id"])
        self.sender.send_depth_uint16(depth_ch, depth, w, h, metadata=meta, group=self.camera["group"],
                                      role="depth", stream_id=self.camera["depth_id"])

    def start(self):
        """None once streaming, else why it cannot."""
        try:
            from ramms_tools.streaming import StreamSender
        except ImportError as exc:
            return f"streaming is unavailable ({exc})"
        try:
            self.sender = StreamSender(host=self.host, port=self.port, queue_size=0)
            self.sender.connect()
        except OSError as exc:
            return f"cannot connect to RMSS on {self.host}:{self.port} ({exc})"
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return None

    def _run(self):
        while not self._stop.is_set():
            frames = self._frames = self._frames or self.frames()
            try:
                self.send_once(*frames)
            except OSError as exc:
                self.error = f"streaming stopped: {exc}"
                return
            self._stop.wait(1.0 / self.rate)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.sender is not None:
            try:
                self.sender.disconnect()  # also unblocks a send the join above timed out on
            except OSError:
                pass


# Runs inside the editor. State lives in a module of its own because each command gets fresh
# globals: a temporary component and any held depth request must survive from one command to the
# next, and be found again (and released) by a later run if this one died half way.
EDITOR_MARKERS_PRELUDE = r'''
import json, sys, types
import unreal

_S = sys.modules.get("_ramms_markers_test")
if _S is None:
    _S = types.ModuleType("_ramms_markers_test")
    _S.component, _S.requested = None, []
    sys.modules["_ramms_markers_test"] = _S


def _world():
    return unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()


def _actors(world, cls=None):
    return list(unreal.GameplayStatics.get_all_actors_of_class(world, cls or unreal.Actor))


def _components(world, cls):
    found = []
    for actor in _actors(world):
        found += list(actor.get_components_by_class(cls))
    return found


def _alive(obj, world):
    try:
        return obj is not None and unreal.SystemLibrary.is_valid(obj) and obj.get_world().get_path_name() == world.get_path_name()
    except Exception:
        return False


def _release():
    for bridge in _S.requested:
        try:
            bridge.release_raw_data_forwarding()
        except Exception:
            pass
    _S.requested = []


def _destroy_own():
    comp, _S.component = _S.component, None
    if comp is not None:
        try:
            comp.set_detected_objects_json('{"objects": []}')
            comp.destroy_component(comp)
        except Exception:
            pass


def _drop_projectors(ids):
    # The level's projection manager gives every stream that turns active a projector of its own,
    # so the test camera would otherwise go on painting its gradient into the 3D view.
    for manager in _components(_world(), unreal.RammsCameraProjectionManager):
        for stream_id in ids:
            try:
                manager.remove_projector(stream_id)
            except Exception:
                pass


def _describe(comp):
    cls = comp.get_editor_property("marker_class")
    try:
        ours = cls is not None and unreal.MathLibrary.class_is_child_of(cls, unreal.RammsDetectionWorldMarker.static_class())
    except Exception:
        ours = False
    return {"path": comp.get_path_name(), "accepted_sources": [str(s) for s in comp.get_editor_property("accepted_sources")],
            "spawn_markers": bool(comp.get_editor_property("spawn_markers")),
            "marker_class": cls.get_name() if cls is not None else "", "marker_is_ramms": bool(ours)}


def _op_setup(params):
    world = _world()
    if world is None:
        return {"error": "no play session: start PIE (Play) first -- the markers live in the game world"}
    # Whatever an earlier run left behind goes first, so this run's measurements are its own.
    _release()
    _destroy_own()
    _drop_projectors(params.get("stream_ids", []))
    level = [c for c in _components(world, unreal.RammsDetectionWorldMarkers) if not c.is_being_destroyed()]
    bridges = _components(world, unreal.RammsStreamCameraBridge)
    host = bridges[0].get_owner() if bridges else _actors(world)[0]
    # The checks' own component, never the level's: it is not registered, so it neither begins play nor
    # binds to bridge pushes, and is driven only by the calls below -- the app's markers are untouched.
    _S.component = unreal.new_object(unreal.RammsDetectionWorldMarkers, outer=host)
    return {"world": world.get_path_name(), "component": _S.component.get_path_name(),
            "level_components": [_describe(c) for c in level],
            "bridges": [b.get_path_name() for b in bridges],
            "sample_radius": int(_S.component.get_editor_property("depth_sample_radius"))}


def _op_derive(params):
    points = []
    for x, y in params["points"]:
        found = _S.component.derive_world_location(params["source"], unreal.Vector2D(x, y))
        points.append(None if found is None else [found.x, found.y, found.z])
    return {"points": points}


def _markers(prefix):
    found = []
    for actor in _actors(_world(), unreal.RammsDetectionWorldMarker):
        label = str(actor.get_label_text())
        if label.startswith(prefix):
            loc = actor.get_actor_location()
            found.append({"label": label, "location": [loc.x, loc.y, loc.z]})
    return found


def _op_apply(params):
    _S.component.set_detected_objects_json(params["payload"])
    return {"markers": _markers(params.get("prefix", ""))}


def _op_markers(params):
    return {"markers": _markers(params.get("prefix", ""))}


def _op_depth(params):
    if params.get("request"):
        for bridge in _components(_world(), unreal.RammsStreamCameraBridge):
            bridge.request_raw_data_forwarding()
            _S.requested.append(bridge)
    else:
        _release()
    return {"held": len(_S.requested)}


def _op_teardown(params):
    _destroy_own()
    _release()
    _drop_projectors(params.get("stream_ids", []))
    return {"ok": True}


def _ramms_markers_run(op, params):
    try:
        result = globals()["_op_" + op](params)
    except Exception as exc:
        result = {"error": "%s: %s" % (type(exc).__name__, exc)}
    print("RAMMS_MARKERS_RESULT " + json.dumps(result))
'''

MARKERS_RESULT = "RAMMS_MARKERS_RESULT "


class EditorMarkers:
    """The world-marker component in the play session, driven through the editor's Python."""

    def __init__(self, editor):
        self.editor = editor

    def call(self, op, **params):
        script = EDITOR_MARKERS_PRELUDE + f"\n_ramms_markers_run({op!r}, json.loads({json.dumps(params)!r}))\n"
        lines = self.editor.run(script)
        result = next((json.loads(line[len(MARKERS_RESULT):]) for line in lines if line.startswith(MARKERS_RESULT)), None)
        if result is None:
            raise BridgeError(f"the editor's {op} step printed no result: {lines[-3:]}")
        if "error" in result:
            raise BridgeError(f"the editor's {op} step failed: {result['error']}")
        return result


def _fmt_point(p):
    return "(" + ", ".join(f"{v:.1f}" for v in p) + ")"


def _off_by(got, want):
    return math.dist(got, want)


def _await_derived(markers, source, point, want, timeout, sleep, clock):
    """The derived position for point once it is within tolerance of want; as it last was at the timeout."""
    deadline = clock() + timeout
    while True:
        got = markers.call("derive", source=source, points=[list(point)])["points"][0]
        if (got is not None and _off_by(got, want) <= MARKER_TOLERANCE_CM) or clock() >= deadline:
            return got
        sleep(0.25)


def marker_payload_cases(camera, depth, radius):
    """(what, object, push stream_id, expected world point or None, why it fails today if it does)."""
    cid = camera["color_id"]
    w, h = camera["size"]
    box_centre, wall = (0.5, 0.5), (0.8, 0.2)
    at_box = expected_derived(*box_centre, sample_depth_cm(depth, *box_centre, radius), camera)
    at_wall = expected_derived(*wall, sample_depth_cm(depth, *wall, radius), camera)

    def obj(key, **fields):
        return dict({"instance_id": MARKER_PREFIX + key, "perception_class": MARKER_PREFIX + key}, **fields)

    sent_camera = (0.10, -0.05, 1.50)
    sent_world = (1.0, 0.5, 0.8)
    return [
        ("centroid as [x, y], normalized, `source` = the camera's stream id",
         obj("array", centroid=[0.5, 0.5], source=cid), "", at_box, ""),
        ("centroid as {\"X\", \"Y\"} -- how the bridge encodes it for UE",
         obj("object", centroid={"X": 0.5, "Y": 0.5}, source=cid), "", at_box,
         "the component reads centroid and box only as JSON arrays, and the bridge re-encodes both as "
         "objects (USER_INPUT_JSON_SCHEMA.md, end of section 3), so it derives nothing for any real bridge object"),
        (f"centroid in pixels of the {w}x{h} frame -- the bridge's units",
         obj("pixels", centroid=[wall[0] * w, wall[1] * h], source=cid), "", at_wall,
         "a centroid is used as if normalized; the bridge sends pixels of the source's frame, which the "
         "camera widget divides by the frame size"),
        ("box only, as [x0, y0, x1, y1]: placed at its centre",
         obj("box_array", box=[0.45, 0.45, 0.55, 0.55], source=cid), "", at_box, ""),
        ("box only, as {\"X\", \"Y\", \"Z\", \"W\"} -- how the bridge encodes it",
         obj("box_object", box={"X": 0.45, "Y": 0.45, "Z": 0.55, "W": 0.55}, source=cid), "", at_box,
         "box is read only as a JSON array, as centroid is"),
        ("no `source` on the object: the push's stream_id names the camera",
         obj("push_source", centroid=[0.5, 0.5]), cid, at_box, ""),
        ("a centroid where the depth has no return: no marker",
         obj("dropout", centroid=[0.22, 0.22], source=cid), "", None, ""),
        ("sent position_m, world frame (the 3D path, as a control)",
         obj("sent_world", position_m=dict(zip("XYZ", sent_world)), frame="world", source=cid), "",
         expected_sent_world(sent_world), ""),
        ("sent position_m, camera frame, through the stream's pose (control)",
         obj("sent_camera", position_m=dict(zip("XYZ", sent_camera)), frame="camera", source=cid), "",
         expected_sent_camera(sent_camera, camera), ""),
    ]


def _check_bound_component(markers, client, check, components):
    """Whether the level's world-marker component places markers from bridge pushes: push one through it.

    A sent world-frame position, so no camera or depth can stand in the way, tagged with a source the
    component accepts. Cleared again by pushing the empty set.
    """
    what = "the level's world-marker component places markers from bridge pushes"
    if not components:
        report([("warn", what, "there is none: no actor in this level carries URammsDetectionWorldMarkers, so it "
                 "shows no world markers at all. The checks below use one of their own, given each payload "
                 "directly -- the receiver passes pushes through unchanged")], check)
        return
    comp = components[0]
    if not comp["spawn_markers"]:
        report([("warn", what, f"not checked: {comp['path']} has bSpawnMarkers off, so what it shows is up to "
                 "whatever listens to OnDetectionWorldPoints")], check)
        return
    if not comp["marker_is_ramms"]:
        report([("warn", what, f"not checked: its MarkerClass ({comp['marker_class'] or 'none'}) is not a "
                 "RammsDetectionWorldMarker, so its markers cannot be read back here")], check)
        return
    accepted = comp["accepted_sources"]
    source = "" if not accepted or "" in accepted else accepted[0]
    probe = {"instance_id": MARKER_PREFIX + "bound", "perception_class": MARKER_PREFIX + "bound",
             "position_m": {"X": 1.0, "Y": 0.5, "Z": 0.8}, "frame": "world", "source": source}
    client.update_detected_objects(detected_objects_payload([probe], "", ""))
    try:
        placed = markers.call("markers", prefix=probe["perception_class"])["markers"]
    finally:
        client.update_detected_objects(detected_objects_payload([], "", ""))
    check(bool(placed), what, f"{comp['path']} placed no marker for a pushed position_m (source {source!r}): "
          "it is not receiving bridge pushes -- not registered, or it never began play")


def _check_pushed_derived(markers, client, check, camera, depth, radius, components, timeout, sleep, clock):
    """A 2D object pushed through the receiver, as the bridge sends one, gets a derived marker from the level's component.

    The app's whole path: the receiver, the bridge subsystem, the level's own component (registered, so
    bound to pushes, and asking for depth itself), the {"X", "Y"} pixel centroid the bridge sends.
    """
    what = "a 2D object pushed through the receiver gets a derived marker from the level's component"
    if not components or not components[0]["spawn_markers"] or not components[0]["marker_is_ramms"]:
        return  # already reported: no level component, or none whose markers can be read back
    accepted = components[0]["accepted_sources"]
    if accepted and not any(a.lower() == camera["color_id"].lower() for a in accepted):
        report([("warn", what, f"not checked: the level's component accepts only {accepted}, not the test camera's "
                 f"'{camera['color_id']}' (`markers --scene` streams it as the scene camera)")], check)
        return
    w, h = camera["size"]
    wall = (0.8, 0.2)
    want = expected_derived(*wall, sample_depth_cm(depth, *wall, radius), camera)
    label = MARKER_PREFIX + "pushed"
    obj = {"instance_id": label, "perception_class": label, "source": camera["color_id"],
           "centroid": {"X": wall[0] * w, "Y": wall[1] * h}}
    client.update_detected_objects(detected_objects_payload([obj], "", ""))
    try:
        deadline = clock() + timeout
        while True:
            placed = markers.call("markers", prefix=label)["markers"]
            if placed or clock() >= deadline:
                break
            sleep(0.25)  # the level's component asks for depth on its first need, and retries meanwhile
    finally:
        client.update_detected_objects(detected_objects_payload([], "", ""))
    if not placed:
        check(False, what, f"no marker within {timeout:g} s")
        return
    off = _off_by(placed[0]["location"], want)
    check(off <= MARKER_TOLERANCE_CM and len(placed) == 1, f"{what}, at {_fmt_point(want)}",
          f"{len(placed)} marker(s), at {_fmt_point(placed[0]['location'])}, {off:.1f} cm away")


def run_marker_checks(markers, client, check, feeder, camera=MARKER_CAMERA, timeout=8.0, sleep=None, clock=None,
                      unaided_box_mm=None):
    """Every derived-position check against the streamed test camera; leaves nothing behind.

    unaided_box_mm: the box's depth while checking what the component manages unaided -- a value this
    run alone uses, so a position derived from an older frame (the provider keeps a stream's last raw
    frame after forwarding stops) cannot pass for a fresh one.
    """
    sleep, clock = sleep or _sleep, clock or _clock
    if unaided_box_mm is None:
        unaided_box_mm = 1300 + int(clock() * 1000) % 400 * 2
    # Projectors are dropped only for the test camera's own ids: under --scene they are the real scene
    # camera's, and its projector is the level's to keep.
    ids = [camera["color_id"], camera["depth_id"]] if camera.get("own_ids", True) else []
    setup = markers.call("setup", stream_ids=ids)
    try:
        _marker_checks(markers, client, check, feeder, camera, setup, timeout, sleep, clock, unaided_box_mm)
    finally:
        feeder.set_box_mm(MARKER_BOX_MM)
        markers.call("teardown", stream_ids=ids)


def _marker_checks(markers, client, check, feeder, camera, setup, timeout, sleep, clock, unaided_box_mm):
    radius = setup["sample_radius"]
    depth = marker_depth_image(camera)
    cid = camera["color_id"]
    _check_bound_component(markers, client, check, setup.get("level_components", []))
    if not setup["bridges"]:
        check(False, "a RammsStreamCameraBridge in the play session",
              "none: nothing turns RMSS frames into camera streams, so nothing can be derived")
        return

    centre = (0.5, 0.5)
    feeder.set_box_mm(unaided_box_mm)
    unaided = marker_depth_image(camera, unaided_box_mm)
    want = expected_derived(*centre, sample_depth_cm(unaided, *centre, radius), camera)
    got = _await_derived(markers, cid, centre, want, min(timeout, 3.0), sleep, clock)
    what = "a derived position from the current depth, with only what the component asks for itself"
    if got is None:
        check(False, what, "none: no depth bytes reach the camera provider. The component has to get the stream "
              "bridge to forward depth (RequestDepthRawDataForwarding) itself; before that fix only "
              "RammsCameraProjectionManager asked, and only for CPU projection. The rest of this run holds a request "
              "of its own")
    else:
        off = _off_by(got, want)
        check(off <= MARKER_TOLERANCE_CM, what,
              f"placed at {_fmt_point(got)}, {off:.1f} cm from where the box is now: derived from an OLD depth "
              "frame. The provider keeps a stream's last raw frame after raw-data forwarding stops (here, an "
              "earlier run's), so markers go on using depth that is no longer arriving")
    feeder.set_box_mm(MARKER_BOX_MM)
    markers.call("depth", request=True)
    want = expected_derived(*centre, sample_depth_cm(depth, *centre, radius), camera)
    got = _await_derived(markers, cid, centre, want, timeout, sleep, clock)
    what = "a derived position from the current depth, with depth forwarded"
    if got is None:
        check(False, what, f"none after {timeout:g} s: '{cid}' was registered earlier with another size or "
              "without intrinsics (restart the play session), or its frames stopped arriving")
        return
    # A position that never settles on the right one, with fresh frames arriving all along, is the
    # projection disagreeing, not a wait that was too short: say so, and let the points below show how.
    off = _off_by(got, want)
    check(off <= MARKER_TOLERANCE_CM, what, f"the image centre settles at {_fmt_point(got)}, {off:.1f} cm from "
          f"{_fmt_point(want)}, with depth forwarded for {timeout:g} s: the projection disagrees (see below)")

    _check_pushed_derived(markers, client, check, camera, depth, radius, setup.get("level_components", []),
                          timeout, sleep, clock)

    points = [("the box in front of the wall, image centre", centre),
              ("the wall, upper right", (0.8, 0.2)),
              ("the wall, top-left pixel", (0.0, 0.0)),
              ("the wall, bottom-right pixel", (319.5 / 320, 239.5 / 240))]
    found = markers.call("derive", source=cid, points=[list(p) for _, p in points])["points"]
    for (what, p), got in zip(points, found):
        want = expected_derived(*p, sample_depth_cm(depth, *p, radius), camera)
        if got is None:
            check(False, f"{what} {p}: derived to {_fmt_point(want)}", "no position at all")
        else:
            off = _off_by(got, want)
            check(off <= MARKER_TOLERANCE_CM, f"{what} {tuple(round(v, 3) for v in p)}: derived to {_fmt_point(want)}",
                  f"placed at {_fmt_point(got)}, {off:.1f} cm away")

    dropout, outside = (0.22, 0.22), (1.25, 0.5)
    got_dropout, got_outside = markers.call("derive", source=cid, points=[list(dropout), list(outside)])["points"]
    check(got_dropout is None, "no position where the depth has no return",
          f"placed at {_fmt_point(got_dropout or (0, 0, 0))}")
    check(got_outside is None, f"no position for a point outside the image {outside}",
          f"placed at {_fmt_point(got_outside or (0, 0, 0))}: depth is read at the clamped edge pixel and the "
          "point unprojected anyway, so a pixel centroid taken as normalized lands metres off rather than nowhere")

    scene_id = _scene_stream_id(client) or "camera/scene/color"
    via_alias, direct = (markers.call("derive", source=s, points=[list(centre)])["points"][0]
                         for s in ("scene", scene_id))
    if cid == scene_id and direct is not None:
        # The test camera is the scene camera (--scene): an object exactly as the bridge sends it.
        w, h = camera["size"]
        wall = (0.8, 0.2)
        want_wall = expected_derived(*wall, sample_depth_cm(depth, *wall, radius), camera)
        obj = {"instance_id": MARKER_PREFIX + "bridge", "perception_class": MARKER_PREFIX + "bridge", "source": "scene",
               "box": {"X": 0.75 * w, "Y": 0.15 * h, "Z": 0.85 * w, "W": 0.25 * h},
               "centroid": {"X": wall[0] * w, "Y": wall[1] * h}}
        placed = markers.call("apply", payload=json.dumps(detected_objects_payload([obj], "", "")),
                              prefix=MARKER_PREFIX)["markers"]
        mine = [m for m in placed if m["label"] == obj["perception_class"]]
        off = _off_by(mine[0]["location"], want_wall) if mine else None
        check(bool(mine) and off <= MARKER_TOLERANCE_CM,
              f"an object exactly as the bridge sends it (source \"scene\", box and centroid as {{\"X\", ...}} pixels): "
              f"marker at {_fmt_point(want_wall)}",
              f"at {_fmt_point(mine[0]['location'])}, {off:.1f} cm away" if mine else "no marker")
    if direct is None:
        report([("warn", "the bridge's source \"scene\" places objects on the scene camera",
                 f"not decided: the scene camera ('{scene_id}') derives nothing either -- nothing streams it, or "
                 "not with intrinsics, a pose and depth (the synthetic video sends neither). `markers --scene` "
                 "streams the test camera as the scene camera, which decides it")], check)
    elif via_alias is None:
        check(False, "the bridge's source \"scene\" places objects on the scene camera",
              f"'{scene_id}' derives {_fmt_point(direct)}, but 'scene' derives nothing: the component looks "
              "`source` up as a stream id, with no alias table like the camera widget's SceneDetectionSources")
    else:
        check(_off_by(via_alias, direct) <= MARKER_TOLERANCE_CM, "the bridge's source \"scene\" places objects on "
              "the scene camera", f"'scene' gives {_fmt_point(via_alias)}, '{scene_id}' gives {_fmt_point(direct)}")

    for what, obj, push_stream, want, why in marker_payload_cases(camera, depth, radius):
        payload = json.dumps(detected_objects_payload([obj], "", push_stream))
        placed = markers.call("apply", payload=payload, prefix=MARKER_PREFIX)["markers"]
        mine = [m for m in placed if m["label"] == obj["perception_class"]]
        others = [m["label"] for m in placed if m["label"] != obj["perception_class"]]
        if want is None:
            check(not mine, what, f"a marker at {_fmt_point(mine[0]['location'])}" if mine else "")
        elif not mine:
            check(False, f"{what}: marker at {_fmt_point(want)}", "no marker" + (f" -- {why}" if why else ""))
        else:
            off = _off_by(mine[0]["location"], want)
            check(off <= MARKER_TOLERANCE_CM and len(mine) == 1, f"{what}: marker at {_fmt_point(want)}",
                  f"{len(mine)} marker(s), at {_fmt_point(mine[0]['location'])}, {off:.1f} cm away"
                  + (f" -- {why}" if why else ""))
        if others:
            check(False, f"{what}: only this push's marker", f"also {others}: earlier markers were not removed")


def cmd_markers(client, args):
    """Check world markers derived from 2D detections against a streamed camera with known geometry."""
    from ramms_tools.editor_exec import EditorExec, EditorExecError
    camera = MARKER_CAMERA_COLOUR_META if getattr(args, "metadata", "depth") == "both" else MARKER_CAMERA
    if getattr(args, "scene", False):
        # The scene camera's own stream ids and group, so `source: "scene"` names this camera. It takes
        # over the scene camera view while it runs; the bridge re-registers the stream at this size.
        scene_color, scene_depth, _ = VideoFeeder(client, args.host, args.stream_port)._settings()
        camera = dict(camera, color_id=scene_color, depth_id=scene_depth, group="camera/scene", channels=(0, 1),
                      own_ids=False)
    feeder = MarkerCameraFeeder(args.host, args.stream_port, camera)
    problem = feeder.start()
    if problem:
        print(f"  cannot stream the test camera: {problem}")
        return 1
    k, t = camera["intrinsics"], camera["transform"]
    print(f"  streaming a {camera['size'][0]}x{camera['size'][1]} test camera on "
          f"{camera['color_id']!r} + {camera['depth_id']!r}: fx={k['fx']:g} fy={k['fy']:g} "
          f"cx={k['cx']:g} cy={k['cy']:g}, at ({t['x']:g}, {t['y']:g}, {t['z']:g}) cm, "
          f"pitch {t['pitch']:g} yaw {t['yaw']:g} roll {t['roll']:g}")
    print(f"  intrinsics and pose on the {'colour and depth' if camera['meta_on'] == 'both' else 'depth'} frames"
          f"{'' if camera['meta_on'] == 'both' else ' only, as the UI bridge sends them'}")
    print(f"  depth: wall {MARKER_WALL_MM / 1000:g} m, a box at {MARKER_BOX_MM / 1000:g} m over {MARKER_BOX}, "
          f"no return over {MARKER_DROPOUT}")
    check, failures = _reporter()
    try:
        with EditorExec(project=args.editor_project) as editor:
            run_marker_checks(EditorMarkers(editor), client, check, feeder, camera=camera, timeout=args.timeout)
    except EditorExecError as exc:
        print(f"  cannot use the editor's Python: {exc}")
        return 1
    except BridgeError as exc:
        check(False, "world-marker checks", str(exc))
    finally:
        feeder.stop()
        if feeder.error:
            print(f"  note: {feeder.error}")
    print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all world-marker checks passed'}")
    return 1 if failures else 0


def _synthetic_frames(t, w, h):
    """The three synthetic channel payloads for the frame at time `t`."""
    import numpy as np

    img = np.zeros((h, w, 4), dtype=np.uint8)
    img[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
    img[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
    img[:, :, 2] = int((t * 60) % 256)
    img[:, :, 3] = 255
    bar = int((t * w / 3.0) % w)
    img[:, max(0, bar - 4):bar + 4, :3] = 255

    depth = np.linspace(300, 4000, h, dtype=np.uint16)[:, None].repeat(w, axis=1)

    overlay = np.zeros((h, w, 4), dtype=np.uint8)
    overlay[:, :, 2] = 255
    overlay[:, :, 3] = 255
    cy, cx = h // 2, int((t * w / 2.0) % w)
    overlay[max(0, cy - 40):cy + 40, max(0, cx - 40):cx + 40, 1] = 255

    return img, depth, overlay


def _channel_sends(sender, t, w, h, ids, overlay):
    """One zero-arg send per channel, in the order they should go out.

    Spacing these across the frame is what keeps the receiver from dropping
    all but the last: its inbound queue is shallow, and a burst arriving
    between two game-thread ticks starves every channel but one.
    """
    scene_color, scene_depth, overlay_color = ids
    img, depth, ov = _synthetic_frames(t, w, h)
    sends = [
        lambda: sender.send_image(0, img.tobytes(), w, h, fmt="bgra8",
                                  group="camera/scene", role="color",
                                  stream_id=scene_color),
        lambda: sender.send_depth_uint16(1, depth.tobytes(), w, h,
                                         group="camera/scene", role="depth",
                                         stream_id=scene_depth),
    ]
    if overlay:
        sends.append(lambda: sender.send_image(2, ov.tobytes(), w, h, fmt="bgra8",
                                               group="overlay", role="color",
                                               stream_id=overlay_color))
    return sends


def _send_frame_spaced(sends, base, period, sleep_until, stop=None):
    """Send each channel, spread evenly across one frame period.

    Scheduled against absolute deadlines derived from `base` rather than by
    sleeping a fixed gap after each send: a fixed gap adds the time spent
    building and sending the frame to every interval, so the stream drifts
    below the rate that was asked for. Exactly one sleep per send, which is
    the other half of getting the rate right.
    """
    slot = period / len(sends)
    for index, send in enumerate(sends):
        if stop is not None and stop():
            return False
        send()
        sleep_until(base + (index + 1) * slot)
    return True


class VideoFeeder:
    """Background synthetic video, so boxes have something to sit on.

    The two layers are independent feeds: the scene camera renders
    `SceneColorStreamID` (channel 0) and the engine overlay renders
    `OverlayColorStreamID` (channel 2). Sending only one leaves the other
    blank no matter what objects are pushed at it, which is the single most
    confusing thing about driving this interface by hand.
    """

    def __init__(self, client, host, port, width=640, height=360, fps=12.0,
                 overlay=True):
        self.client = client
        self.host, self.port = host, port
        self.width, self.height = width, height
        self.fps = fps if fps > 0 else 12.0
        self.overlay = overlay
        self._thread = None
        self._stop = threading.Event()
        self.error = None

    def _settings(self):
        def setting(name, default):
            try:
                return self.client.get_setting(name) or default
            except BridgeError:
                return default

        scene_color = setting("SceneColorStreamID", "camera/scene/color")
        overlay_color = setting("OverlayColorStreamID", "engine/overlay")
        data = setting("SceneDataStreams", []) or []
        scene_depth = next((d.get("StreamID") for d in data
                            if d.get("Kind") == "Depth"), "camera/scene/depth")
        return scene_color, scene_depth, overlay_color

    def _run(self):
        try:
            from ramms_tools.streaming import StreamSender
        except ImportError as exc:
            self.error = f"no video (needs numpy): {exc}"
            return
        ids = self._settings()
        w, h = self.width, self.height
        sender = StreamSender(host=self.host, port=self.port)
        try:
            sender.connect()
        except OSError as exc:
            self.error = f"no video (RMSS {self.host}:{self.port}): {exc}"
            return

        period = 1.0 / self.fps
        start = time.monotonic()
        frame = 0
        try:
            while not self._stop.is_set():
                t = time.monotonic() - start
                sends = _channel_sends(sender, t, w, h, ids, self.overlay)
                _send_frame_spaced(
                    sends,
                    start + frame * period,
                    period,
                    lambda deadline: self._stop.wait(max(0.0, deadline - time.monotonic())),
                    stop=self._stop.is_set,
                )
                frame += 1
        except OSError as exc:
            # Queued sends are reported by the worker, but anything raised here
            # would otherwise end the thread in silence and look like "video is
            # running" for the rest of the walkthrough.
            self.error = f"video stopped: {exc}"
        finally:
            try:
                sender.disconnect(flush=True)
            except Exception:  # noqa: BLE001 - teardown must not mask the real result
                pass

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        time.sleep(1.0)  # let a first frame land before anything is pushed
        return self.error

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)


def cmd_stream(client, args):
    try:
        import numpy as np

        from ramms_tools.streaming import StreamSender
    except ImportError as exc:
        # numpy is a declared dependency, so this only bites when running
        # straight from a source checkout via PYTHONPATH, where nothing
        # resolved it. Every other command here is stdlib-only.
        print(f"the stream command needs numpy, which is not importable here: {exc}")
        print("  pip install ramms-tools   (or: pip install numpy)")
        return 2

    def setting(name, default):
        try:
            return client.get_setting(name) or default
        except BridgeError:
            return default

    scene_color = setting("SceneColorStreamID", "camera/scene/color")
    overlay_color = setting("OverlayColorStreamID", "engine/overlay")
    scene_data = setting("SceneDataStreams", []) or []
    scene_depth = next((d.get("StreamID") for d in scene_data
                        if d.get("Kind") == "Depth"), "camera/scene/depth")

    w, h = args.width, args.height
    # queue_size=0 sends synchronously, so a socket error raises here instead of
    # being logged by a worker thread while this command still prints
    # "sent N frames ..." and exits 0. This is a diagnostic: it should fail
    # loudly. (It still only proves the bytes left the socket, not that the
    # editor rendered them -- that part is eyes-only.)
    sender = StreamSender(host=args.host, port=args.stream_port, queue_size=0)
    print(f"connecting to RMSS at {args.host}:{args.stream_port} ...")
    try:
        sender.connect()
    except OSError as exc:
        # Usually the editor is not in PIE: RammsStreamReceiver only listens
        # while the game world exists.
        print(f"  cannot reach the RMSS server: {exc}")
        print("  is PIE running? the stream receiver only listens during play.")
        return 2
    print(f"  scene colour -> ch0 {scene_color!r}")
    print(f"  scene depth  -> ch1 {scene_depth!r}")
    if args.overlay:
        print(f"  overlay      -> ch2 {overlay_color!r}")
    print(f"expect: a {w}x{h} animated gradient with a moving bar, and depth that "
          "ramps front-to-back.")

    ids = (scene_color, scene_depth, overlay_color)
    period = 1.0 / args.fps
    frames = 0
    start = time.monotonic()

    def sleep_until(deadline):
        time.sleep(max(0.0, deadline - time.monotonic()))

    try:
        while time.monotonic() - start < args.seconds:
            t = time.monotonic() - start
            sends = _channel_sends(sender, t, w, h, ids, args.overlay)
            _send_frame_spaced(sends, start + frames * period, period, sleep_until)
            frames += 1
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        sender.disconnect(flush=True)
    elapsed = time.monotonic() - start
    # No drop counter here: nothing is queued, so a frame either went out or
    # raised.
    print(f"sent {frames} frames in {elapsed:.1f}s ({frames / max(elapsed, 1e-6):.1f} fps)")
    return 0


def cmd_selftest(client, args):
    """Transport-level checks that need no human. Exits non-zero on failure."""
    failures = []

    def check(ok, what, detail=""):
        print(f"  {'pass' if ok else 'FAIL'}  {what}{(' -- ' + detail) if detail and not ok else ''}")
        if not ok:
            failures.append(what)

    print("preset introspection")
    try:
        client.load_preset()
        found = set(client._labels)
        want = set(FALLBACK_LABELS)
        check(want <= found, "preset exposes all six bridge functions",
              f"missing {sorted(want - found)}")
        check(client._arguments.get("UpdatePanel") == ["PanelJSON"],
              "UpdatePanel declares PanelJSON",
              str(client._arguments.get("UpdatePanel")))
        unbound = client.unbound_functions() or []
        check(not unbound, "every exposed function is bound to an object",
              f"{unbound} reach nothing -- Remote Control still answers HTTP 200, "
              "so a push to them would be reported as a success. Rebind them in "
              "the open level.")
        # Only the bridge's own entries make the checks below meaningless. A dangling
        # joint or system-state entry (the robot display was replaced, not the receiver)
        # is a failure in its own right, but must not hide every bridge check.
        bridge_unbound = [f for f in unbound if f in FALLBACK_LABELS]
        if bridge_unbound:
            print("\nFAILED: the bridge functions are not bound; later checks would be "
                  "meaningless, so they are skipped")
            return 1
    except BridgeError as exc:
        check(False, "preset readable", str(exc))
        return 1

    print("parameter-name guard")
    try:
        client.call("UpdatePanel", NotAParameter="{}")
        check(False, "an unknown parameter name is rejected",
              "it was sent to UE, which accepts the call and receives an empty "
              "string -- clearing the panel instead of updating it")
    except BridgeError:
        check(True, "an unknown parameter name is rejected before sending")
    try:
        # Case does not matter to UE: PanelJSON/PanelJson/paneljson all land.
        # Refusing these would block calls that work.
        client.call("UpdatePanel", PanelJson=json.dumps(PANEL_SCENARIOS["idle"][0]))
        check(True, "a differently-cased parameter name is accepted")
    except BridgeError as exc:
        check(False, "a differently-cased parameter name is accepted", str(exc))

    print("pushes accepted")
    for name in PANEL_SCENARIOS:
        try:
            client.update_panel(PANEL_SCENARIOS[name][0])
            check(True, f"UpdatePanel [{name}]")
        except BridgeError as exc:
            check(False, f"UpdatePanel [{name}]", str(exc))
    for source in ("scene", "overlay"):
        try:
            client.update_detected_objects(
                detected_objects_payload(sample_objects(source), "cup_1", ""))
            check(True, f"UpdateDetectedObjects [{source}]")
        except BridgeError as exc:
            check(False, f"UpdateDetectedObjects [{source}]", str(exc))
    for fn, label in ((client.update_world_state, "UpdateWorldState"),
                      (client.open_camera_feed, "OpenCameraFeed"),
                      (client.close_camera_feed, "CloseCameraFeed")):
        try:
            fn({"objects": []}) if label == "UpdateWorldState" else fn()
            check(True, label)
        except BridgeError as exc:
            check(False, label, str(exc))

    print("event channel")
    try:
        latest, events = client.poll_events(0)
        check(isinstance(latest, int), "PollUIEvents returns a sequence")
        check(all("event_type" in e for e in events),
              "every buffered event has an event_type")
        high, tail = client.poll_events(latest)
        check(tail == [], "polling from the latest sequence returns nothing new",
              f"got {len(tail)}")
        check(high >= latest, "sequence does not go backwards")
    except BridgeError as exc:
        check(False, "PollUIEvents", str(exc))

    print("button styles")
    try:
        hud = find_game_instance(client)
        if hud is None:
            print("  skip  no HUD found: the style and POI checks read widgets in a running game")
        else:
            for scenario in ("styles", "style-fallback"):
                run_style_checks(client, hud, scenario, check)
            print("POI layer")
            run_poi_checks(client, hud, check)
    except BridgeError as exc:
        check(False, "HUD readback", str(exc))

    print("joints")
    skipped = None
    if JOINT_FUNCTION not in client._labels:
        skipped = f"{JOINT_FUNCTION} is not on this preset"
        print(f"  skip  {skipped}")
    else:
        try:
            rig = JointRig.locate(client)
        except JointsUnavailable as exc:
            skipped = str(exc)
            print(f"  skip  {skipped}")
        except BridgeError as exc:
            check(False, "the robot display's joints are reachable", str(exc))
        else:
            check(True, f"the robot display's joints are reachable ({rig.world})")
            run_joint_checks(rig, check)

    # Leave the UI somewhere sane rather than on the stress panel.
    try:
        client.update_panel(PANEL_SCENARIOS["idle"][0])
        client.update_detected_objects(detected_objects_payload([], "", ""))
    except BridgeError:
        pass

    print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all checks passed'}"
          + (f" -- joints NOT checked: {skipped}" if skipped else ""))
    return 1 if failures else 0


def cmd_walkthrough(client, args):
    """Guided pass: push each surface, pause, and confirm taps round-trip.

    Streams synthetic video on both layers throughout, because detection boxes
    are drawn over a camera's image: with nothing streaming, the scene camera
    and the engine overlay have no frames, and objects pushed at either look
    like they did not arrive.
    """
    feeder = None
    if not args.no_video:
        feeder = VideoFeeder(client, args.host, args.stream_port)
        problem = feeder.start()
        print(f"  {problem}" if problem else
              "  streaming synthetic video on the scene and overlay layers")
    try:
        return _walkthrough_steps(client, args)
    finally:
        if feeder:
            feeder.stop()
            if feeder.error:
                print(f"  note: {feeder.error}")


def _optional(call, name):
    """Run a bridge call whose absence should not end the walkthrough.

    The camera feed is optional throughout this tool -- REQUIRED_FUNCTIONS deliberately
    leaves OpenCameraFeed/CloseCameraFeed out, and selftest degrades gracefully for them.
    A preset without them must not abort the run before the steps that exercise clearing,
    which is what the later steps are for.
    """
    def run():
        try:
            call()
        except BridgeError as exc:
            print(f"  SKIPPED: {name} is not on this preset ({exc})")
            print("           the overlay will not open, so the overlay steps below show nothing")
    return run


def _push_progress(client, args, pushes):
    """Push a run of progress panels, spaced so a human can watch the bar move."""
    for i, (pan, pct) in enumerate(pushes):
        client.update_panel(pan)
        print(f"  pushed pct={pct:.2f}")
        if i + 1 < len(pushes):
            time.sleep(args.progress_interval)


class _WalkthroughJoints:
    """Joint steps for the walkthrough, skipped as a group when the preset has no joints."""

    def __init__(self, client):
        self.client = client
        self.rig = None
        self.original = None
        self.unavailable = None

    def step(self, action):
        def run():
            if self.unavailable is None and self.rig is None:
                try:
                    self.rig = JointRig.locate(self.client)
                    self.original = self.rig.targets()
                except JointsUnavailable as exc:
                    self.unavailable = str(exc)
            if self.unavailable:
                print(f"  SKIPPED: {self.unavailable}")
                return
            action(self.rig)
            state = self.rig.arm_state() if self.rig.monitor else None
            if state is not None:
                print(f"  monitor: arm {state}, wheels {'moving' if self.rig.wheels_moving() else 'still'}")
        return run

    def restore(self):
        if self.rig and self.original:
            self.rig.restore(self.original)


def _drive_and_park(rig, rate=10.0, step=5.0, frames=10):
    wheels = rig.monitored("WheelJointNames")
    base = rig.targets()
    held = {n: base[rig.index[n]] for n in wheels}
    seen = False
    for k in range(1, frames + 1):
        rig.send(with_joints(base, rig.index, {n: v + k * step for n, v in held.items()}))
        seen = seen or rig.wheels_moving()
        _sleep(1.0 / rate)
    if not seen:
        raise BridgeError(f"{frames} frames of +{step} deg never registered as the wheels moving")
    idle = float(rig.monitor["WheelIdleTimeout"])
    _sleep(idle)
    moving = _poll_until(rig.wheels_moving, False, idle + 1.0, _sleep, _clock)
    if moving:
        raise BridgeError("the wheels stopped changing but the monitor still says moving")


def _walkthrough_styles(client, scenario):
    root = find_game_instance(client)
    if root is None:
        client.update_panel(PANEL_SCENARIOS[scenario][0])
        print("  (no HUD found to read the buttons back: judge by eye)")
        return
    check, failures = _reporter()
    run_style_checks(client, root, scenario, check)
    if failures:
        print(f"  {len(failures)} style check(s) failed -- see above")


def _walkthrough_poi_tap(client, args):
    root = find_game_instance(client)
    if root is None:
        print("  SKIPPED: no HUD found to read the POIs back")
        return
    ns = argparse.Namespace(tap=not args.no_taps, timeout=args.timeout)
    cmd_pois(client, ns)


NEXT_BUTTON = "walkthrough:next"
NEXT_PANEL = panel(
    title="Walkthrough",
    message="Tap Next, or any box or POI on screen, to go on to the next step.",
    buttons=[button(NEXT_BUTTON, "Next")],
)
# Where nothing on screen can move a step on (--no-taps, or no terminal to watch alongside the
# taps), the panel says what does, rather than offering a Next that would be ignored.
ENTER_PANEL = panel(title="Walkthrough", message="Press Enter in the terminal to go on to the next step.")


class StepTaps(NamedTuple):
    """What on screen moves a walkthrough step on, besides Enter.

    checked: the tap is part of what the step checks (a panel's own buttons), so the
    wait is bounded by --timeout and a miss is reported. Otherwise the step waits, as
    long as it takes, for a tap or Enter -- the time is for looking, not for tapping.

    rebaseline: the step's action waits for taps of its own, so only what comes after it
    may move the step on -- otherwise the tap the action just used would, at once.
    """
    buttons: tuple = ()
    objects: tuple = ()
    checked: bool = False
    rebaseline: bool = False


def _button_ids(*panels):
    """Every button id on these panels, in order, each once."""
    return tuple(dict.fromkeys(b["button_id"] for pan in panels for b in pan.get("buttons", [])))


def _panel_taps(name):
    return StepTaps(buttons=_button_ids(PANEL_SCENARIOS[name][0]), checked=True)


def _next_taps(*object_lists):
    """Next on NEXT_PANEL, or a tap on any of these objects that has an id."""
    ids = tuple(dict.fromkeys(o["instance_id"] for objs in object_lists for o in objs if o.get("instance_id")))
    return StepTaps(buttons=(NEXT_BUTTON,), objects=ids)


def _step_prompt(taps):
    ways = []
    if NEXT_BUTTON in taps.buttons:
        ways.append("Next on the panel")
    others = [b for b in taps.buttons if b != NEXT_BUTTON]
    if others:
        ways.append(f"a button ({', '.join(others)})")
    if taps.objects:
        ways.append(f"a box or POI ({', '.join(taps.objects)})")
    if not ways:
        return "  press Enter to go on (nothing on screen reports a tap at this step)"
    return f"  to go on, tap {' or '.join(ways)} -- or press Enter"


def _await_step(client, taps, since, timeout=None, sleep=None, clock=None):
    """Wait for a press of one of taps.buttons, a tap on one of taps.objects, or Enter.

    Returns how the step was moved on: "button", "object", "enter" or "timeout".
    Other events are printed and do not move it on. An object tap without an id is
    flagged: the UI should not post one.
    """
    sleep, clock = sleep or _sleep, clock or _clock
    deadline = None if timeout is None else clock() + timeout
    while deadline is None or clock() < deadline:
        _, events = client.poll_events(since)
        for event in events:
            since = max(since, int(event.get("seq", since)))
            print(f"    [{event.get('seq')}] {_describe(event)}")
            kind = event.get("event_type")
            if kind == "button_press" and event.get("button_id") in taps.buttons:
                print(f"  OK: {event.get('button_id')!r} round-tripped")
                return "button"
            if kind == "object_tapped" and not event.get("instance_id"):
                print("  note: a tap posted object_tapped with no instance_id -- an object without")
                print("        an id should send nothing")
            elif kind == "object_tapped" and event.get("instance_id") in taps.objects:
                print(f"  OK: {event.get('instance_id')!r} round-tripped")
                return "object"
        if _enter_pressed():
            return "enter"
        sleep(0.4)
    wanted = list(taps.buttons) + list(taps.objects)
    print(f"  no matching tap within {timeout:.0f}s (wanted one of {wanted}); going on")
    return "timeout"


def _walkthrough_poi_step(client, args, screen_moves_on):
    """Put Next on the panel for the rest of the walkthrough (or say Enter), then check the POIs."""
    client.update_panel(NEXT_PANEL if screen_moves_on else ENTER_PANEL)
    _walkthrough_poi_tap(client, args)


def _walkthrough_steps(client, args):
    joints = _WalkthroughJoints(client)
    scene, scene_b = sample_objects("scene"), sample_objects("scene", variant="b")
    # The same test run_steps makes for the steps after the POI one, which are all unchecked.
    screen_moves_on = not args.no_taps and _can_poll_stdin()
    steps = [
        ("Idle panel", lambda: client.update_panel(PANEL_SCENARIOS["idle"][0]),
         PANEL_SCENARIOS["idle"][1], StepTaps()),
        ("Selection panel", lambda: client.update_panel(PANEL_SCENARIOS["selection"][0]),
         PANEL_SCENARIOS["selection"][1], _panel_taps("selection")),
        ("Progress panel", lambda: client.update_panel(PANEL_SCENARIOS["progress"][0]),
         PANEL_SCENARIOS["progress"][1], _panel_taps("progress")),
        ("Progress advances", lambda: _push_progress(client, args,
            [(progress_panel(pct), pct) for pct in PROGRESS_STEPS]),
         f"The SAME panel at {PROGRESS_STEPS}: the bar moves 0% to 100% while the\n"
         "          title, message and buttons stay put. It should update in place --\n"
         "          a panel that visibly rebuilds, or a bar that only moves on the first\n"
         "          update, is the failure here. Note 0.0 must still SHOW a bar: a bar\n"
         "          that appears only from the second update means something is inferring\n"
         "          \"has progress\" from the value instead of from the key.",
         StepTaps(buttons=_button_ids(progress_panel(0.0)))),
        ("Progress on both scales", lambda: _push_progress(client, args,
            [(progress_panel(pct), pct)
             for pair in PROGRESS_SCALE_PAIRS for pct in pair]),
         "Each value PAIRED across the two scales: 0.25 then 25, 0.65 then 65,\n"
         "          1.0 then 100. The bar must NOT move between the two halves of a\n"
         "          pair. The schema's convention is 0.0-1.0 but fridge_ui_demo yields\n"
         "          0-100 for this widget, so the panel accepts either -- over 1 is a\n"
         "          percentage, 1 or less a fraction. A bar that jumps between a pair\n"
         "          handles only one scale; note 1.0 means FULL here, not 1%.",
         StepTaps(buttons=_button_ids(progress_panel(0.0)))),
        ("Progress across panels", lambda: _push_progress(client, args,
            [(pan, pan["progress"]["pct"]) for pan in PROGRESS_PANELS]),
         "FOUR DIFFERENT panels, each carrying its own progress: 15% unlatching,\n"
         "          55% swinging, 80% paused (Resume replaces Pause), then 45% on a\n"
         "          different title. Each must show its own fraction -- a bar still\n"
         "          holding the previous panel's value, or stuck on the first panel it\n"
         "          saw, is the failure.", StepTaps(buttons=_button_ids(*PROGRESS_PANELS))),
        ("Progress gone", lambda: client.update_panel(PANEL_SCENARIOS["selection"][0]),
         "The bar DISAPPEARS and the selection panel's buttons are back. This is the\n"
         "          regression check for the hasprogress bug: the schema omits the\n"
         "          progress key rather than sending null, so a bar that stays visible\n"
         "          here means the flag is latched instead of following the key.",
         _panel_taps("selection")),
        ("Error panel", lambda: client.update_panel(PANEL_SCENARIOS["error"][0]),
         PANEL_SCENARIOS["error"][1], _panel_taps("error")),
        ("Layout stress", lambda: client.update_panel(PANEL_SCENARIOS["stress"][0]),
         PANEL_SCENARIOS["stress"][1], _panel_taps("stress")),
        ("Button styles", lambda: _walkthrough_styles(client, "styles"),
         PANEL_SCENARIOS["styles"][1] + "\n          Tap \"unavailable\": today it still reports a press.",
         _panel_taps("styles")),
        ("Style fallback", lambda: _walkthrough_styles(client, "style-fallback"),
         PANEL_SCENARIOS["style-fallback"][1], _panel_taps("style-fallback")),
        ("A POI tap dims the others", lambda: _walkthrough_poi_step(client, args, screen_moves_on),
         ("The POIs were read back above (one per object, all opaque); --no-taps\n"
          "          skipped the tap itself." if args.no_taps else
          "The tapped POI stayed bright and every other POI dropped to the layer's\n"
          "          UnselectedPOIOpacity; tapping empty background brought them all back.\n"
          "          Both were read back above. If you hid the scene camera to see the\n"
          "          POIs, press the status bar's camera toggle again: the next steps\n"
          "          draw boxes on it.\n" + (
          "          From here on the panel shows one Next button. It, or a tap on any box\n"
          "          or POI on screen, moves the walkthrough on. A POI tap also dims the\n"
          "          other POIs until you tap the background -- that is the feature."
          if screen_moves_on else
          "          From here on, press Enter in the terminal to go on.")),
         _next_taps(scene)._replace(rebaseline=True)),
        ("Detected objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "cup_1",
                                     _scene_stream_id(client))),
         "Three boxes on the SCENE camera, cup_1 highlighted, and a POI for each\n"
         "          on the scene camera's POI layer. The coordinates are normalized, so\n"
         "          they do not move with the frame size -- but the scene CAMERA is\n"
         "          collapsed until its first frame arrives, so with nothing streaming\n"
         "          expect the POIs alone and no boxes. (The overlay is not open yet;\n"
         "          that is checked at \"Both cameras at once\".)", _next_taps(scene)),
        ("Selection moves", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "bowl_2",
                                     _scene_stream_id(client))),
         "Same three boxes, highlight moves to bowl_2.", _next_taps(scene)),
        ("Objects replaced", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="b"), "jug_5",
                                     _scene_stream_id(client))),
         "A DIFFERENT three objects -- jug_5, plate_6, knife_7 -- with jug_5\n"
         "          highlighted. None of cup_1, bowl_2 or tap_3 may still be on\n"
         "          screen: six at once means the layer adds without removing.", _next_taps(scene_b)),
        ("One object dropped", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="b", count=1), "",
                                     _scene_stream_id(client))),
         "Exactly ONE object, jug_5, nothing highlighted. plate_6 and knife_7 are\n"
         "          gone -- each push is the full set, so what it leaves out is over.",
         _next_taps(scene_b[:1])),
        ("Objects without ids", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="unkeyed"), "",
                                     _scene_stream_id(client))),
         "Two objects labelled \"unknown\", and jug_5 gone. These carry no\n"
         "          instance_id, so a tap on one should send nothing at all: a box's\n"
         "          marker refuses to post without an id. Their POIs do not refuse today\n"
         "          -- tap one with the scene camera hidden, and the walkthrough flags the\n"
         "          object_tapped it posts." + (" Tap Next to go on." if screen_moves_on else ""), _next_taps()),
        ("Objects cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _scene_stream_id(client))),
         "Every box and POI disappears -- not just the scene camera's. An empty\n"
         "          list is the full set, so it clears every stream.", _next_taps()),
        ("Overlay camera opened", _optional(client.open_camera_feed, "OpenCameraFeed"),
         "The engine overlay's window appears. OpenCameraFeed drives this, so it\n"
         "          has to come BEFORE the overlay's objects: boxes and POIs are drawn\n"
         "          over an image, and until that image is up there is nothing to draw\n"
         "          them on.", _next_taps()),
        ("Overlay objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("overlay"), "bowl_2",
                                     _overlay_stream_id(client))),
         "Boxes and POIs now on the ENGINE OVERLAY only, bowl_2 highlighted, and\n"
         "          the scene camera stays empty. The two cameras are independent\n"
         "          surfaces: each needs its own frames, its own objects, and its own\n"
         "          POI layer, because a centroid is normalized to the image it was\n"
         "          found in.", _next_taps(sample_objects("overlay"))),
        ("Overlay cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _overlay_stream_id(client))),
         "The overlay's boxes and POIs disappear. (An empty list clears every\n"
         "          stream; only the overlay had anything to lose here.)", _next_taps()),
        ("Both cameras at once", lambda: client.update_detected_objects(
            detected_objects_payload(
                sample_objects("scene") + sample_objects("overlay", variant="b"),
                "cup_1", _scene_stream_id(client))),
         "ONE push, both cameras: cup_1/bowl_2/tap_3 on the scene camera and\n"
         "          jug_5/plate_6/knife_7 on the engine overlay, each with its own\n"
         "          boxes and POIs. Each object's own `source` decides which surface\n"
         "          takes it, not the push's stream_id. An object must NOT appear on\n"
         "          both, and must not appear on the wrong one: that would mean a\n"
         "          surface is accepting every source and placing another camera's\n"
         "          coordinates against its own rectangle.",
         _next_taps(scene, sample_objects("overlay", variant="b"))),
        ("Cleared the way the bridge clears", lambda: client.update_detected_objects(
            detected_objects_payload([], "", "")),
         "EVERYTHING disappears, from both layers at once. This is the real\n"
         "          bridge's clear: an empty objects list with an EMPTY stream_id.\n"
         "          stream_id is never consulted on a clear, so it clears every\n"
         "          stream -- boxes and POIs alike.", _next_taps()),
        ("Overlay camera closed", _optional(client.close_camera_feed, "CloseCameraFeed"),
         "The engine overlay's window disappears. Its POIs and boxes go with it.", _next_taps()),
        ("Arm stowed", joints.step(lambda rig: rig.send(rig.poses()[0])),
         "The 3D robot's arm folds into its stowed pose beside the seat, and the\n"
         "          status bar's arm text reads Stowed. The joint targets were read back,\n"
         "          so this also proves the push reached the running display.", _next_taps()),
        ("Arm active", joints.step(lambda rig: rig.send(rig.poses()[1])),
         "ONE arm joint swings well out of the stowed pose and the arm text reads\n"
         "          Active. The rest of the arm, the wheels and the gripper do not move:\n"
         "          the push carries every joint, and only one of them changed.", _next_taps()),
        ("Arm stowed again", joints.step(lambda rig: rig.send(rig.poses()[0])),
         "The arm returns and the text reads Stowed again. Two real transitions,\n"
         "          because the monitor only announces a change.", _next_taps()),
        ("Wheels drive, then park", joints.step(lambda rig: _drive_and_park(rig)),
         "While that ran, the drive wheels turned and the chair text read DRIVING;\n"
         "          it now reads PARKED, because the wheel targets stopped changing.", _next_taps()),
    ]

    def run_steps():
        for index, (title, action, expectation, taps) in enumerate(steps, 1):
            print(f"\n[{index}/{len(steps)}] {title}")
            # Taken before the push, so a tap that lands while the step is still pushing counts.
            since = None if args.no_taps else _current_sequence(client)
            try:
                action()
            except BridgeError as exc:
                print(f"  FAILED: {exc}")
                return 1
            if taps.rebaseline and since is not None:
                since = _current_sequence(client)
            print(f"  expect: {expectation}")
            if args.no_taps or not (_can_poll_stdin() or taps.checked):
                # No taps wanted, or no terminal to notice Enter in while also watching for taps.
                input("  press Enter to continue ")
            elif _can_poll_stdin():
                print(_step_prompt(taps))
                _await_step(client, taps, since, args.timeout if taps.checked else None)
            else:
                print(f"  tap any of {list(taps.buttons)} (no tty: waiting up to {args.timeout:.0f}s)")
                _await_step(client, taps, since, args.timeout)
        return 0

    try:
        result = run_steps()
    finally:
        # However the run ended, leave the UI somewhere sane rather than on whatever
        # panel and detections it had reached.
        try:
            client.update_detected_objects(detected_objects_payload([], "", ""))
            client.update_panel(PANEL_SCENARIOS["idle"][0])
        except BridgeError:
            pass
        try:
            joints.restore()
        except BridgeError as exc:
            print(f"  note: could not restore the joint targets: {exc}")
    print("\nwalkthrough complete; UI left on the idle panel"
          if result == 0 else "\nwalkthrough stopped; UI reset to the idle panel")
    return result


# ── helpers ──────────────────────────────────────────────────────────

def _scene_stream_id(client):
    try:
        return client.get_setting("SceneColorStreamID") or ""
    except BridgeError:
        return ""


def _overlay_stream_id(client):
    try:
        return client.get_setting("OverlayColorStreamID") or ""
    except BridgeError:
        return ""


def _event_id(event):
    return event.get("button_id") or event.get("instance_id") or ""


def _describe(event):
    kind = event.get("event_type", "?")
    if kind == "button_press":
        return f"button_press {event.get('button_id')!r}"
    if kind == "object_tapped":
        where = event.get("stream_id")
        text = f"object_tapped {event.get('instance_id')!r}" + (f" on {where!r}" if where else "")
        # A tap on a camera image also says where it landed, in that frame's pixels; a tap in the
        # 3D view leaves both keys out.
        point, size = event.get("tap_point"), event.get("image_size")
        if isinstance(point, list) and len(point) == 2:
            text += f" at ({point[0]:.1f}, {point[1]:.1f})"
            if isinstance(size, list) and len(size) == 2:
                text += f" of {size[0]}x{size[1]}"
        return text + f" [{tap_origin(event)}]"
    return json.dumps(event)


def _current_sequence(client):
    """The latest event sequence, to use as a baseline before pushing."""
    latest, _ = client.poll_events(0)
    return latest


def _await(client, wanted, timeout, kind, allow_skip=False, since=None):
    """Wait for an event of `kind` carrying one of `wanted`.

    Callers capture `since` *before* the push they are waiting on: a tap that
    lands between the push and this call would otherwise sit behind the
    baseline and never be seen, timing out a wait that actually succeeded.
    It is not reset to 0 either, which would accept a stale tap from an
    earlier step.
    """
    if since is None:
        since = _current_sequence(client)
    deadline = time.monotonic() + timeout
    # Enter-to-skip needs a terminal to poll; piped stdin just waits the timeout out.
    can_skip = allow_skip and _can_poll_stdin()
    if allow_skip:
        print("  waiting ..." + ("" if can_skip else " (no tty: waiting for the timeout)"))
    while time.monotonic() < deadline:
        _, events = client.poll_events(since)
        for event in events:
            since = max(since, int(event.get("seq", since)))
            print(f"    [{event.get('seq')}] {_describe(event)}")
            if event.get("event_type") == kind and _event_id(event) in wanted:
                print(f"  OK: {_event_id(event)!r} round-tripped")
                return 0
        if can_skip and _enter_pressed():
            print("  skipped")
            return 0
        time.sleep(0.4)
    print(f"  no matching {kind} within {timeout:.0f}s (wanted one of {wanted})")
    return 0 if allow_skip else 1


def _await_button(client, ids, timeout, allow_skip=False, since=None):
    return _await(client, set(ids), timeout, "button_press", allow_skip, since)


def _await_tap(client, ids, timeout, since=None):
    return _await(client, set(ids), timeout, "object_tapped", since=since)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT,
                        help="Remote Control HTTP port (default 30010)")
    parser.add_argument("--stream-port", type=int, default=DEFAULT_STREAM_PORT,
                        help="RMSS streaming port (default 30030)")
    parser.add_argument("--preset", default=PRESET)
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("discover", help="show what the preset exposes and the live stream IDs")
    sub.add_parser("selftest", help="non-interactive transport checks")

    p = sub.add_parser("panel", help="push one panel scenario")
    p.add_argument("--scenario", choices=sorted(PANEL_SCENARIOS), default="selection")
    p.add_argument("--pct", type=_percent, default=None,
                   help="push the progress panel at this pct (0.0-1.0 or 0-100)")
    p.add_argument("--advance", action="store_true",
                   help=f"walk ONE panel through {PROGRESS_STEPS}, changing only pct")
    p.add_argument("--phases", action="store_true",
                   help="walk several DIFFERENT panels that each carry their own progress")
    p.add_argument("--scales", action="store_true",
                   help="push each pct as a 0.0-1.0 fraction then as 0-100; the bar must not move")
    p.add_argument("--interval", type=_positive_float, default=1.0,
                   help="seconds between --advance updates")
    p.add_argument("--wait", action="store_true", help="wait for a button press")
    p.add_argument("--timeout", type=float, default=60.0)

    p = sub.add_parser("objects", help="push detected objects")
    p.add_argument("--source", choices=("scene", "overlay"), default="scene")
    p.add_argument("--select", default="", help="instance_id to highlight")
    p.add_argument("--stream-id", default=None,
                   help="override the stream_id; pass '' for the empty one the bridge sends")
    p.add_argument("--set", choices=sorted(SAMPLE_SETS), default="a",
                   help="which sample set: 'b' is disjoint from 'a', 'unkeyed' has no instance_ids")
    p.add_argument("--count", type=_positive_int, default=None,
                   help="push only the first N of the set, to check the rest are removed")
    p.add_argument("--clear", action="store_true", help="push an empty list instead")
    p.add_argument("--pixels", action="store_true",
                   help="send pixel-space boxes for a 1280x720 feed instead of normalized 0-1")
    p.add_argument("--wait", action="store_true", help="wait for an object tap")
    p.add_argument("--timeout", type=float, default=60.0)

    sub.add_parser("worldstate", help="push a sample world state")

    p = sub.add_parser("camera", help="open or close the camera feed")
    p.add_argument("state", choices=("open", "close"))

    p = sub.add_parser("stream", help="send synthetic RMSS camera frames")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--fps", type=_positive_float, default=15.0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--overlay", action="store_true", help="also send the engine overlay layer")

    p = sub.add_parser("events", help="watch UI events coming back")
    p.add_argument("--follow", action="store_true")
    p.add_argument("--since", type=int, default=None, help="0 replays the buffer")
    p.add_argument("--expect", default="", help="exit 0 only once this id arrives")
    p.add_argument("--timeout", type=float, default=0.0)
    p.add_argument("--interval", type=float, default=0.4)

    p = sub.add_parser("styles", help="push a button-style panel and check what each button draws")
    p.add_argument("--scenario", choices=["styles", "style-fallback", "stress", "error"], default="styles")
    p.add_argument("--wait", action="store_true", help="then wait for a tap on one of its buttons")
    p.add_argument("--timeout", type=_positive_float, default=60.0)

    p = sub.add_parser("pois", help="push objects to the POI layer and check the POIs (and tap dimming)")
    p.add_argument("--tap", action="store_true", help="ask for a tap on a POI, then on the background, "
                   "and check the dimming each time")
    p.add_argument("--timeout", type=_positive_float, default=60.0)

    p = sub.add_parser("markers", help="world markers derived from 2D: stream a camera with known geometry "
                       "and check where its centroids land (editor play session, via the editor's Python)")
    p.add_argument("--editor-project", default=os.environ.get("RAMMS_EDITOR_PROJECT", "RammpUI"),
                   help="the editor's project to run the checks in (default RammpUI; '*' takes any)")
    p.add_argument("--timeout", type=_positive_float, default=8.0,
                   help="how long to wait for the streamed camera to give a position (default 8 s)")
    p.add_argument("--metadata", choices=("depth", "both"), default="depth",
                   help="which frames carry the camera's intrinsics and pose: depth only, as the UI bridge "
                   "sends them (default), or colour and depth")
    p.add_argument("--scene", action="store_true",
                   help="stream the test camera AS the scene camera (its stream ids), so objects tagged "
                   "source \"scene\" -- as the bridge tags them -- are checked too; takes over the scene view")

    p = sub.add_parser("joints", help="drive the robot display's arm and wheels; check the indicators")
    act = p.add_mutually_exclusive_group()
    act.add_argument("--check", action="store_true",
                     help="stow, activate, stow, drive, park -- each confirmed by readback; restores after")
    act.add_argument("--stow", action="store_true", help="arm to the monitor's stow pose, wheels held")
    act.add_argument("--activate", action="store_true", help="one arm joint well past the stow tolerance")
    act.add_argument("--drive", action="store_true", help="turn the drive wheels for --seconds, then stop")
    act.add_argument("--sweep", action="store_true", help="swing the arm stowed -> active -> stowed, for eyes")
    act.add_argument("--set", nargs="+", metavar="NAME=DEG", help="set joints by name, everything else held")
    p.add_argument("--seconds", type=_positive_float, default=1.0)
    p.add_argument("--rate", type=_positive_float, default=10.0, help="frames per second for --drive/--sweep")
    p.add_argument("--step", type=_positive_float, default=5.0, help="wheel degrees per --drive frame")

    p = sub.add_parser("walkthrough", help="guided pass over every surface")
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--progress-interval", type=_positive_float, default=0.8,
                   help="seconds between the progress updates in the progress steps")
    p.add_argument("--no-taps", action="store_true", help="never wait for a tap")
    p.add_argument("--no-video", action="store_true",
                   help="do not stream synthetic frames during the walkthrough")

    args = parser.parse_args()
    client = BridgeClient(args.host, args.port, preset=args.preset, verbose=args.verbose)

    handlers = {
        "discover": cmd_discover, "selftest": cmd_selftest, "panel": cmd_panel,
        "objects": cmd_objects, "worldstate": cmd_worldstate, "camera": cmd_camera,
        "stream": cmd_stream, "events": cmd_events, "joints": cmd_joints,
        "styles": cmd_styles, "pois": cmd_pois, "markers": cmd_markers,
        "walkthrough": cmd_walkthrough,
    }
    try:
        return handlers[args.command](client, args)
    except BridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
