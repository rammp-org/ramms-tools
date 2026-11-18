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

Not automatic: whether a panel *looks* right. Nothing readable comes back for
panel/world-state/camera-feed state, so each scenario prints what you should
see and, where it matters, waits for you to tap and confirms the ID round-trips.
"""

import argparse
import json
import os
import select
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

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
            parsed = json.loads(body)
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
            raise BridgeError(
                f"{function} is exposed on preset {self.preset} but bound to no "
                "object, so the call would return HTTP 200 and reach nothing. "
                "Rebind the preset entries to the RammsBridgeReceiver in the "
                "level that is currently open, then re-run this command -- the "
                "preset is read once per run."
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
            progress={"phase": "opening", "pct": 0.4, "message": "opening door",
                      "pausable": True},
        ),
        "A progress bar at 40% (pct is a 0.0-1.0 fraction) plus Pause/Cancel.",
    ),
    "paused": (
        panel(
            title="fridge_ui_demo: open",
            message="paused",
            buttons=[button("resume", "Resume", "normal"), button("cancel", "Cancel", "warning")],
            progress={"phase": "opening", "pct": 0.4, "message": "paused", "pausable": False},
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
        "Panel-level error styling plus an error-styled button.",
    ),
    "stress": (
        panel(
            title="A title that is quite a lot longer than any real panel would ever use",
            message=("Long body text to check wrapping and clipping. " * 6
                     + "Unicode: café — ½° ✓ 日本語"),
            buttons=[button(f"many:{i}", f"Button {i}", ["normal", "warning", "error"][i % 3])
                     for i in range(8)],
        ),
        "Long text and 8 buttons -- check wrapping, clipping and layout.",
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
        print(f"  {name:<24} label={label!r:<28} args={argnames}")
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
        print("  It happens when the level holding RammsBridgeReceiver is moved or")
        print("  replaced: the preset still points at the old path. Rebind the")
        print("  preset entries to the receiver in the level that is open now.")
        return 1

    latest, _ = client.poll_events(0)
    print(f"\nUI event sequence is at {latest}")
    return 0


def cmd_panel(client, args):
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
              "so every push below would be reported as a success. Rebind the "
              "preset to the receiver in the open level.")
        if unbound:
            print("\nFAILED: the preset is not bound; later checks would be "
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

    # Leave the UI somewhere sane rather than on the stress panel.
    try:
        client.update_panel(PANEL_SCENARIOS["idle"][0])
        client.update_detected_objects(detected_objects_payload([], "", ""))
    except BridgeError:
        pass

    print(f"\n{'FAILED: ' + str(len(failures)) if failures else 'all checks passed'}")
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


def _walkthrough_steps(client, args):
    steps = [
        ("Idle panel", lambda: client.update_panel(PANEL_SCENARIOS["idle"][0]),
         PANEL_SCENARIOS["idle"][1], []),
        ("Selection panel", lambda: client.update_panel(PANEL_SCENARIOS["selection"][0]),
         PANEL_SCENARIOS["selection"][1], scenario_buttons("selection")),
        ("Progress panel", lambda: client.update_panel(PANEL_SCENARIOS["progress"][0]),
         PANEL_SCENARIOS["progress"][1], scenario_buttons("progress")),
        ("Error panel", lambda: client.update_panel(PANEL_SCENARIOS["error"][0]),
         PANEL_SCENARIOS["error"][1], scenario_buttons("error")),
        ("Layout stress", lambda: client.update_panel(PANEL_SCENARIOS["stress"][0]),
         PANEL_SCENARIOS["stress"][1], scenario_buttons("stress")),
        ("Detected objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "cup_1",
                                     _scene_stream_id(client))),
         "Three boxes on the SCENE camera, cup_1 highlighted, and a POI for each\n"
         "          on the scene camera's POI layer. The coordinates are normalized, so\n"
         "          they do not move with the frame size -- but the scene CAMERA is\n"
         "          collapsed until its first frame arrives, so with nothing streaming\n"
         "          expect the POIs alone and no boxes. (The overlay is not open yet;\n"
         "          that is checked at \"Both cameras at once\".)", []),
        ("Selection moves", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "bowl_2",
                                     _scene_stream_id(client))),
         "Same three boxes, highlight moves to bowl_2.", []),
        ("Objects replaced", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="b"), "jug_5",
                                     _scene_stream_id(client))),
         "A DIFFERENT three objects -- jug_5, plate_6, knife_7 -- with jug_5\n"
         "          highlighted. None of cup_1, bowl_2 or tap_3 may still be on\n"
         "          screen: six at once means the layer adds without removing.", []),
        ("One object dropped", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="b", count=1), "",
                                     _scene_stream_id(client))),
         "Exactly ONE object, jug_5, nothing highlighted. plate_6 and knife_7 are\n"
         "          gone -- each push is the full set, so what it leaves out is over.", []),
        ("Objects without ids", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene", variant="unkeyed"), "",
                                     _scene_stream_id(client))),
         "Two objects labelled \"unknown\", and jug_5 gone. These carry no\n"
         "          instance_id, so nothing can key a marker on them and a tap on one\n"
         "          sends nothing at all -- the marker refuses to post without an id.", []),
        ("Objects cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _scene_stream_id(client))),
         "Every box and POI disappears -- not just the scene camera's. An empty\n"
         "          list is the full set, so it clears every stream.", []),
        ("Overlay camera opened", _optional(client.open_camera_feed, "OpenCameraFeed"),
         "The engine overlay's window appears. OpenCameraFeed drives this, so it\n"
         "          has to come BEFORE the overlay's objects: boxes and POIs are drawn\n"
         "          over an image, and until that image is up there is nothing to draw\n"
         "          them on.", []),
        ("Overlay objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("overlay"), "bowl_2",
                                     _overlay_stream_id(client))),
         "Boxes and POIs now on the ENGINE OVERLAY only, bowl_2 highlighted, and\n"
         "          the scene camera stays empty. The two cameras are independent\n"
         "          surfaces: each needs its own frames, its own objects, and its own\n"
         "          POI layer, because a centroid is normalized to the image it was\n"
         "          found in.", []),
        ("Overlay cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _overlay_stream_id(client))),
         "The overlay's boxes and POIs disappear. (An empty list clears every\n"
         "          stream; only the overlay had anything to lose here.)", []),
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
         "          coordinates against its own rectangle.", []),
        ("Cleared the way the bridge clears", lambda: client.update_detected_objects(
            detected_objects_payload([], "", "")),
         "EVERYTHING disappears, from both layers at once. This is the real\n"
         "          bridge's clear: an empty objects list with an EMPTY stream_id.\n"
         "          stream_id is never consulted on a clear, so it clears every\n"
         "          stream -- boxes and POIs alike.", []),
        ("Overlay camera closed", _optional(client.close_camera_feed, "CloseCameraFeed"),
         "The engine overlay's window disappears. Its POIs and boxes go with it.", []),
    ]

    def run_steps():
        for index, (title, action, expectation, ids) in enumerate(steps, 1):
            print(f"\n[{index}/{len(steps)}] {title}")
            since = _current_sequence(client) if (ids and not args.no_taps) else None
            try:
                action()
            except BridgeError as exc:
                print(f"  FAILED: {exc}")
                return 1
            print(f"  expect: {expectation}")
            if ids and not args.no_taps:
                print(f"  tap any of {ids} (or press Enter to skip)")
                if _await_button(client, ids, args.timeout, allow_skip=True,
                                 since=since) == 1:
                    return 1
            else:
                input("  press Enter to continue ")
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
        return f"object_tapped {event.get('instance_id')!r}" + (f" on {where!r}" if where else "")
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
    # Enter-to-skip needs a selectable stdin, which Windows does not give us for
    # console handles; there we just wait the timeout out.
    can_skip = allow_skip and os.name != "nt" and sys.stdin is not None and sys.stdin.isatty()
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
        if can_skip and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
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

    p = sub.add_parser("walkthrough", help="guided pass over every surface")
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--no-taps", action="store_true", help="never wait for a tap")
    p.add_argument("--no-video", action="store_true",
                   help="do not stream synthetic frames during the walkthrough")

    args = parser.parse_args()
    client = BridgeClient(args.host, args.port, preset=args.preset, verbose=args.verbose)

    handlers = {
        "discover": cmd_discover, "selftest": cmd_selftest, "panel": cmd_panel,
        "objects": cmd_objects, "worldstate": cmd_worldstate, "camera": cmd_camera,
        "stream": cmd_stream, "events": cmd_events, "walkthrough": cmd_walkthrough,
    }
    try:
        return handlers[args.command](client, args)
    except BridgeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
