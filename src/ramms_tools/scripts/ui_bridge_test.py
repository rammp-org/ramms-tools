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
the RCP_RammsBridge preset loaded -- i.e. a level containing RammsBridgeReceiver.

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
PRESET = "RCP_RammsBridge"
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


class BridgeError(RuntimeError):
    pass


class BridgeClient:
    """Remote Control HTTP client for the RCP_RammsBridge preset."""

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_HTTP_PORT, timeout=8.0,
                 preset=PRESET, verbose=False):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout
        self.preset = preset
        self.verbose = verbose
        self._labels = {}    # C++ name -> exposed label
        self._arguments = {}  # C++ name -> [argument names]

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

    def load_preset(self):
        """Map each C++ function name to its exposed label and argument names."""
        data = self._request("GET", f"/remote/preset/{self.preset}")
        preset = data.get("Preset", data)
        for group in preset.get("Groups", []):
            for fn in group.get("ExposedFunctions", []):
                underlying = fn.get("UnderlyingFunction", {})
                name = underlying.get("Name")
                if not name:
                    continue
                self._labels[name] = fn.get("DisplayName") or name
                self._arguments[name] = [a.get("Name") for a in underlying.get("Arguments", [])]
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

        UE matches parameters by name and silently ignores ones it does not
        recognise, so a typo shows up as an empty string on the UE side rather
        than an error. Catch that here instead.
        """
        declared = self._arguments.get(function)
        if declared is None and not self._labels:
            try:
                self.load_preset()
                declared = self._arguments.get(function)
            except BridgeError:
                declared = None
        if declared is not None:
            unknown = sorted(set(parameters) - set(declared))
            if unknown:
                raise BridgeError(
                    f"{function} does not take {unknown}; it declares {declared}. "
                    "UE would accept the call and receive an empty value."
                )
        label = urllib.parse.quote(self.label_for(function))
        payload = {"Parameters": parameters, "GenerateTransaction": False}
        if self.verbose:
            print(f"  -> {function}({', '.join(parameters)})")
        result = self._request("PUT", f"/remote/preset/{self.preset}/function/{label}", payload)
        returned = (result or {}).get("ReturnedValues") or [{}]
        return returned[0]

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


def sample_objects(source, pixels=False):
    """Three tappable objects, normalized unless `pixels` asks otherwise."""
    samples = PIXEL_SAMPLES if pixels else NORMALIZED_SAMPLES
    return [detected_object(iid, cls, box, centroid, source)
            for iid, cls, box, centroid in samples]


# ── commands ─────────────────────────────────────────────────────────

def cmd_discover(client, args):
    preset = client.load_preset()
    print(f"preset: {preset.get('Name', PRESET)}")
    print("\nexposed functions (call by LABEL, not by C++ name):")
    for name, label in sorted(client._labels.items()):
        argnames = client._arguments.get(name) or []
        print(f"  {name:<24} label={label!r:<28} args={argnames}")
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

    latest, _ = client.poll_events(0)
    print(f"\nUI event sequence is at {latest}")
    return 0


def cmd_panel(client, args):
    pan, expectation = PANEL_SCENARIOS[args.scenario]
    client.update_panel(pan)
    print(f"pushed panel scenario {args.scenario!r}")
    print(f"  expect: {expectation}")
    ids = scenario_buttons(args.scenario)
    print(f"  button_ids: {ids or '(none)'}")
    if args.wait and ids:
        return _await_button(client, ids, args.timeout)
    return 0


def cmd_objects(client, args):
    stream_id = args.stream_id
    if not stream_id:
        prop = "SceneColorStreamID" if args.source == "scene" else "OverlayColorStreamID"
        try:
            stream_id = client.get_setting(prop) or ""
        except BridgeError:
            stream_id = ""
    objects = [] if args.clear else sample_objects(args.source, args.pixels)
    payload = detected_objects_payload(objects, args.select or "", stream_id)
    client.update_detected_objects(payload)
    if args.clear:
        print(f"cleared detected objects on stream_id={stream_id!r}")
        print("  expect: every box disappears from the view.")
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
        return _await_tap(client, [o["instance_id"] for o in objects], args.timeout)
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
        return 0


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
        self.width, self.height, self.fps = width, height, fps
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
            import numpy as np

            from ramms_tools.streaming import StreamSender
        except ImportError as exc:
            self.error = f"no video (needs numpy): {exc}"
            return
        scene_color, scene_depth, overlay_color = self._settings()
        w, h = self.width, self.height
        sender = StreamSender(host=self.host, port=self.port)
        try:
            sender.connect()
        except OSError as exc:
            self.error = f"no video (RMSS {self.host}:{self.port}): {exc}"
            return
        start = time.monotonic()
        try:
            while not self._stop.is_set():
                t = time.monotonic() - start
                img = np.zeros((h, w, 4), dtype=np.uint8)
                img[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
                img[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
                img[:, :, 2] = int((t * 60) % 256)
                img[:, :, 3] = 255
                bar = int((t * w / 3.0) % w)
                img[:, max(0, bar - 4):bar + 4, :3] = 255
                sender.send_image(0, img.tobytes(), w, h, fmt="bgra8",
                                  group="camera/scene", role="color", stream_id=scene_color)
                depth = np.linspace(300, 4000, h, dtype=np.uint16)[:, None].repeat(w, axis=1)
                sender.send_depth_uint16(1, depth.tobytes(), w, h, group="camera/scene",
                                         role="depth", stream_id=scene_depth)
                if self.overlay:
                    ov = np.zeros((h, w, 4), dtype=np.uint8)
                    ov[:, :, 2] = 255
                    ov[:, :, 3] = 255
                    cy, cx = h // 2, int((t * w / 2.0) % w)
                    ov[max(0, cy - 40):cy + 40, max(0, cx - 40):cx + 40, 1] = 255
                    sender.send_image(2, ov.tobytes(), w, h, fmt="bgra8", group="overlay",
                                      role="color", stream_id=overlay_color)
                self._stop.wait(max(0.0, 1.0 / self.fps))
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
    sender = StreamSender(host=args.host, port=args.stream_port)
    print(f"connecting to RMSS at {args.host}:{args.stream_port} ...")
    sender.connect()
    print(f"  scene colour -> ch0 {scene_color!r}")
    print(f"  scene depth  -> ch1 {scene_depth!r}")
    if args.overlay:
        print(f"  overlay      -> ch2 {overlay_color!r}")
    print(f"expect: a {w}x{h} animated gradient with a moving bar, and depth that "
          "ramps front-to-back.")

    frames = 0
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            t = time.monotonic() - start
            # BGRA gradient with a bar sweeping left to right.
            img = np.zeros((h, w, 4), dtype=np.uint8)
            img[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
            img[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
            img[:, :, 2] = int((t * 60) % 256)
            img[:, :, 3] = 255
            bar = int((t * w / 3.0) % w)
            img[:, max(0, bar - 4):bar + 4, :3] = 255
            sender.send_image(0, img.tobytes(), w, h, fmt="bgra8",
                              group="camera/scene", role="color", stream_id=scene_color)

            # uint16 depth in millimetres, near at the top.
            depth = np.linspace(300, 4000, h, dtype=np.uint16)[:, None].repeat(w, axis=1)
            sender.send_depth_uint16(1, depth.tobytes(), w, h,
                                     group="camera/scene", role="depth", stream_id=scene_depth)

            if args.overlay:
                ov = np.zeros((h, w, 4), dtype=np.uint8)
                ov[:, :, 2] = 255
                ov[:, :, 3] = 255
                cy, cx = h // 2, int((t * w / 2.0) % w)
                ov[max(0, cy - 40):cy + 40, max(0, cx - 40):cx + 40, 1] = 255
                sender.send_image(2, ov.tobytes(), w, h, fmt="bgra8",
                                  group="overlay", role="color", stream_id=overlay_color)

            frames += 1
            time.sleep(max(0.0, 1.0 / args.fps))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        sender.disconnect(flush=True)
    elapsed = time.monotonic() - start
    print(f"sent {frames} frames in {elapsed:.1f}s ({frames / max(elapsed, 1e-6):.1f} fps), "
          f"dropped {sender.dropped_frames}")
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
    except BridgeError as exc:
        check(False, "preset readable", str(exc))
        return 1

    print("parameter-name guard")
    try:
        client.call("UpdatePanel", PanelJson="{}")
        check(False, "a mis-cased parameter name is rejected",
              "the wrong name was sent to UE, which would receive an empty string")
    except BridgeError:
        check(True, "a mis-cased parameter name is rejected before sending")

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
        ("Camera feed open", client.open_camera_feed,
         "The camera feed appears.", []),
        ("Detected objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "cup_1",
                                     _scene_stream_id(client))),
         "Three boxes on the scene view, cup_1 highlighted. Coordinates are\n          normalized, so they land the same whether or not a feed is streaming.", []),
        ("Selection moves", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("scene"), "bowl_2",
                                     _scene_stream_id(client))),
         "Same three boxes, highlight moves to bowl_2.", []),
        ("Objects cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _scene_stream_id(client))),
         "All boxes disappear from the scene camera.", []),
        ("Overlay objects", lambda: client.update_detected_objects(
            detected_objects_payload(sample_objects("overlay"), "bowl_2",
                                     _overlay_stream_id(client))),
         "Boxes now on the ENGINE OVERLAY, not the scene camera, bowl_2\n"
         "          highlighted. The two layers are independent: each needs its own\n"
         "          frames and its own objects.", []),
        ("Overlay cleared", lambda: client.update_detected_objects(
            detected_objects_payload([], "", _overlay_stream_id(client))),
         "The overlay's boxes disappear.", []),
        ("Camera feed close", client.close_camera_feed,
         "The camera feed disappears.", []),
    ]

    for index, (title, action, expectation, ids) in enumerate(steps, 1):
        print(f"\n[{index}/{len(steps)}] {title}")
        try:
            action()
        except BridgeError as exc:
            print(f"  FAILED: {exc}")
            return 1
        print(f"  expect: {expectation}")
        if ids and not args.no_taps:
            print(f"  tap any of {ids} (or press Enter to skip)")
            if _await_button(client, ids, args.timeout, allow_skip=True) == 1:
                return 1
        else:
            input("  press Enter to continue ")

    client.update_panel(PANEL_SCENARIOS["idle"][0])
    print("\nwalkthrough complete; UI left on the idle panel")
    return 0


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


def _await(client, wanted, timeout, kind, allow_skip=False):
    since, _ = client.poll_events(0)
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


def _await_button(client, ids, timeout, allow_skip=False):
    return _await(client, set(ids), timeout, "button_press", allow_skip)


def _await_tap(client, ids, timeout):
    return _await(client, set(ids), timeout, "object_tapped")


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
    p.add_argument("--stream-id", default="", help="override the stream_id")
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
    p.add_argument("--fps", type=float, default=15.0)
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
