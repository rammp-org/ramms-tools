"""Offline tests for the HUD readback in ramms_tools.scripts.ui_bridge_test: what the panel's
buttons draw for each style, and how the POI layer dims around a tap.

FakeHUD models RammpUI as measured in PIE and read from its Blueprint graphs on 2026-11-20
(ramms-ui 4769575):

- RAMMS_Panel builds each pushed panel's buttons TWICE (its own OnPanelJson binding and W_RAMMS_UI's
  UpdatePanel call). The first set is taken off the panel at once and lingers, with an empty Slot,
  until garbage collection.
- Buttons are RAMMS_Button_C_<k> under the panel's WidgetTree_0, k counting up across pushes. Their
  PanelButton struct's field names carry Blueprint GUID suffixes.
- RAMMS_Button picks its look with a case-insensitive switch on valid | invalid | error | cancel |
  unavailable, and draws anything else -- normal, warning, absent, unknown -- as valid. Its cancel
  branch is unwired, so a cancel button keeps its designer look -- measured identical to invalid --
  and reads "OPEN".
- POIs are RAMMS_POI_C_<k> under the game instance, one per instance_id (a new id gets a new k; a
  released POI lingers with an empty Slot and its old object); id-less objects reuse POIs in order.
  A tap posts object_tapped with neither stream_id nor tap_point, and dims every other POI to
  UnselectedPOIOpacity until a tap on the background clears it.

The switches (cancel_wired, warning_distinct, ...) turn individual behaviours on or off, so each
check is shown to pass on a correct UI and to fail on the defect it is there for.
"""
from __future__ import annotations

import copy
import json
import urllib.parse

import pytest

from ramms_tools.scripts import ui_bridge_test as ubt
from ramms_tools.scripts.ui_bridge_test import (
    PANEL_SCENARIOS,
    BridgeClient,
    BridgeError,
    check_button_styles,
    check_poi_dimming,
    check_pois,
    find_game_instance,
    read_panel_buttons,
    read_pois,
    sample_objects,
    tap_origin,
    ui_button_style,
    unmangle,
)

ROOT = "/Engine/Transient.UnrealEdEngine_0:GameInstance_3"
PANEL = ROOT + ".W_RAMMS_UI_C_0.WidgetTree_0.RAMMS_Panel.WidgetTree_0"
LAYER = ROOT + ".W_RAMMS_UI_C_0.WidgetTree_0.RammsDetectionPOILayer"
ARM_STATUS = ROOT + ".W_RAMMS_UI_C_0.WidgetTree_0.RAMMS_StatusBar.WidgetTree_0.TXT_ArmStatus"
GUID = "2C79449C41E72E24CFD65B9933A7F2C7"
BRIDGE_ARGS = {"UpdatePanel": ["PanelJSON"], "UpdateDetectedObjects": ["DetectedObjectsJSON"],
               "UpdateWorldState": ["WorldStateJSON"], "PollUIEvents": ["SinceSequence"],
               "OpenCameraFeed": [], "CloseCameraFeed": []}

BLUE, AMBER, NAVY, ORANGE = (0.07, 0.144, 0.451, 1.0), (0.89, 0.45, 0.03, 1.0), (0.019, 0.023, 0.038, 1.0), \
    (0.95, 0.35, 0.0, 1.0)
LOOKS = {  # style -> (tint, label opacity, rich-text tag)
    "valid": (BLUE, 1.0, "btn-light"),
    "invalid": (AMBER, 1.0, "btn-dark"),
    "error": (NAVY, 1.0, "btn-light"),
    "unavailable": (BLUE, 0.5, "btn-light"),
    "cancel": ((0.8, 0.1, 0.1, 1.0), 1.0, "btn-light"),
    "warning": ((0.9, 0.6, 0.1, 1.0), 1.0, "btn-dark"),  # only with warning_distinct
}


class Clock:
    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeHUD:
    def __init__(self, cancel_wired=False, warning_distinct=False, case_sensitive=False, error_looks_valid=False,
                 duplicate_live=False, start_k=0, poi_dims=True, leave_released_pois=False, start_poi_k=0,
                 build_late=False, dim_without_tap=False, duplicate_poi=False):
        self.cancel_wired = cancel_wired
        self.warning_distinct = warning_distinct
        self.case_sensitive = case_sensitive
        self.error_looks_valid = error_looks_valid
        self.duplicate_live = duplicate_live
        self.poi_dims = poi_dims
        self.leave_released_pois = leave_released_pois
        self.next_poi_k = start_poi_k
        self.poi_by_id = {}     # instance_id -> path
        self.unkeyed = []       # id-less POI paths, reused in order
        self.build_late = build_late          # the live buttons get their Slot one read later
        self.pending_slots = []
        self.late_armed = False
        self.dim_without_tap = dim_without_tap
        self.duplicate_poi = duplicate_poi
        self.background_requested = False     # set when the tool asks for a background tap
        self.objects = {ARM_STATUS: {"Text": "<status-default>Active</>"},
                        LAYER: {"UnselectedPOIOpacity": 0.4, "Visibility": "SelfHitTestInvisible"}}
        self.next_k = start_k
        self.last_tapped = None
        self.events = []
        self.on_poll = None     # called on every PollUIEvents (an operator tapping meanwhile)
        self.answers_background = False      # the operator taps the background when asked

    # ── panel ───────────────────────────────────────────────────

    def _look(self, style):
        key = style if self.case_sensitive else (style or "").lower()
        if key == "warning" and self.warning_distinct:
            return LOOKS["warning"], True
        if key not in ("valid", "invalid", "error", "cancel", "unavailable"):
            key = "valid"
        if key == "cancel" and not self.cancel_wired:
            return LOOKS["invalid"], False  # the designer default (measured: same as invalid); label not set
        if key == "error" and self.error_looks_valid:
            key = "valid"
        return LOOKS[key], True

    def _make_button(self, b, live):
        path = f"{PANEL}.RAMMS_Button_C_{self.next_k}"
        self.next_k += 1
        (tint, opacity, tag), labelled = self._look(b.get("style"))
        text = f"<{tag}>{b['label'] if labelled else 'OPEN'}</>"
        self.objects[path] = {
            "PanelButton": {f"button_id_2_{GUID}": b["button_id"], f"label_4_{GUID}": b["label"],
                            f"style_6_{GUID}": b.get("style", "")},
            "Slot": f"{PANEL}.BOX_Commands.HorizontalBoxSlot_{self.next_k}" if live and not self.build_late else "",
        }
        if live and self.build_late:
            self.pending_slots.append(path)
        self.objects[path + ".WidgetTree_0.TXT_Button"] = {"Text": text, "RenderOpacity": opacity}
        self.objects[path + ".WidgetTree_0.BTN"] = {"WidgetStyle": {"Normal": {"TintColor": {
            "SpecifiedColor": dict(zip("RGBA", tint))}}}}

    def update_panel(self, pan):
        for path, props in self.objects.items():
            if ".RAMMS_Button_C_" in path and "Slot" in props:
                props["Slot"] = ""
        for live in (False, True):  # built twice; the first set is taken off the panel at once
            for b in pan.get("buttons", []):
                self._make_button(b, live)
        if self.duplicate_live and pan.get("buttons"):
            self._make_button(pan["buttons"][0], True)

    # ── POIs ────────────────────────────────────────────────────

    def _new_poi(self):
        path = f"{ROOT}.RAMMS_POI_C_{self.next_poi_k}"
        self.next_poi_k += 1
        self.objects[path] = {"DetectedObjectData": {}, "Slot": "", "ColorAndOpacity": {"R": 1, "G": 1, "B": 1, "A": 1}}
        return path

    def update_objects(self, payload):
        objects = payload.get("objects", [])
        live = []
        ids = {o.get("instance_id") for o in objects if o.get("instance_id")}
        for instance_id, path in list(self.poi_by_id.items()):
            if instance_id not in ids:
                del self.poi_by_id[instance_id]
                if not self.leave_released_pois:
                    self.objects[path]["Slot"] = ""   # released: off the canvas, object kept
        unkeyed = iter(self.unkeyed)
        used_unkeyed = []
        for i, o in enumerate(objects):
            instance_id = o.get("instance_id", "")
            if instance_id:
                path = self.poi_by_id.get(instance_id) or self._new_poi()
                self.poi_by_id[instance_id] = path
            else:
                path = next(unkeyed, None) or self._new_poi()
                used_unkeyed.append(path)
            props = self.objects[path]
            props["DetectedObjectData"] = {"InstanceId": instance_id, "PerceptionClass": o.get("perception_class", "")}
            props["Slot"] = f"{LAYER}.WidgetTree.POICanvas.CanvasPanelSlot_{i}"
            live.append(path)
        for path in self.unkeyed:
            if path not in used_unkeyed and not self.leave_released_pois:
                self.objects[path]["Slot"] = ""
        self.unkeyed = used_unkeyed + [p for p in self.unkeyed if p not in used_unkeyed]
        if self.duplicate_poi and objects and objects[0].get("instance_id"):
            dup = self._new_poi()
            self.objects[dup]["DetectedObjectData"] = {"InstanceId": objects[0]["instance_id"], "PerceptionClass": ""}
            self.objects[dup]["Slot"] = f"{LAYER}.WidgetTree.POICanvas.CanvasPanelSlot_dup"
        self._apply_opacity()

    def _live_pois(self):
        return [props for path, props in self.objects.items() if ".RAMMS_POI_C_" in path and props["Slot"]]

    def _apply_opacity(self):
        ids = [p["DetectedObjectData"].get("InstanceId") for p in self._live_pois()]
        dimming = self.poi_dims and self.last_tapped in ids and self.last_tapped
        for props in self._live_pois():
            mine = props["DetectedObjectData"].get("InstanceId") == self.last_tapped
            props["ColorAndOpacity"]["A"] = 0.4 if self.dim_without_tap or (dimming and not mine) else 1.0

    def tap_poi(self, instance_id):
        self.last_tapped = instance_id
        self._apply_opacity()
        self.post({"event_type": "object_tapped", "instance_id": instance_id})

    def tap_background(self):
        self.last_tapped = None
        self._apply_opacity()

    # ── events ──────────────────────────────────────────────────

    def post(self, event):
        self.events.append(dict(event, seq=len(self.events) + 1))

    # ── Remote Control ──────────────────────────────────────────

    def read(self, path, prop):
        if path in self.objects and prop in self.objects[path]:
            return copy.deepcopy(self.objects[path][prop])
        raise BridgeError(f"Object: {path} could not be resolved")

    def preset(self):
        return {"Preset": {"Name": "RCPS", "Groups": [{"Name": "All", "ExposedProperties": [], "ExposedFunctions": [
            {"DisplayName": label, "UnderlyingFunction": {"Name": name, "Arguments": [{"Name": a} for a in BRIDGE_ARGS[name]]},
             "OwnerObjects": [{"Path": "/Game/Maps/Map_UI.Map_UI:PersistentLevel.RammsBridgeReceiver_1"}]}
            for name, label in ubt.FALLBACK_LABELS.items()]}]}}

    def request(self, method, endpoint, payload=None):
        if method == "GET" and endpoint == "/remote/presets":
            return {"Presets": [{"Name": "RCPS"}]}
        if method == "GET" and endpoint == "/remote/preset/RCPS":
            return self.preset()
        if endpoint == "/remote/object/property":
            return {payload["propertyName"]: self.read(payload["objectPath"], payload["propertyName"])}
        if endpoint == "/remote/batch":
            props_read = {r["Body"]["propertyName"] for r in payload["Requests"]}
            if self.pending_slots and self.late_armed and "PanelButton" in props_read:
                for path in self.pending_slots:  # built late: on the panel from the NEXT read-back on
                    self.objects[path]["Slot"] = f"{PANEL}.BOX_Commands.HorizontalBoxSlot_late"
                self.pending_slots = []
            if self.pending_slots and "Slot" in props_read:
                self.late_armed = True           # one whole read-back has now seen them missing
            if self.background_requested and self.last_tapped and \
                    any(r["Body"]["propertyName"] == "ColorAndOpacity" for r in payload["Requests"]):
                self.tap_background()            # the operator taps the background once asked
            out = []
            for r in payload["Requests"]:
                try:
                    body = {r["Body"]["propertyName"]: self.read(r["Body"]["objectPath"], r["Body"]["propertyName"])}
                    out.append({"RequestId": r["RequestId"], "ResponseCode": 200, "ResponseBody": body})
                except BridgeError as exc:
                    out.append({"RequestId": r["RequestId"], "ResponseCode": 400, "ResponseBody": {"errorMessage": str(exc)}})
            return {"Responses": out}
        prefix = "/remote/preset/RCPS/function/"
        if endpoint.startswith(prefix):
            label = urllib.parse.unquote(endpoint[len(prefix):])
            # UE matches parameter names ignoring case (selftest sends PanelJson on purpose).
            params = {key[:1].upper() + key[1:]: value for key, value in payload["Parameters"].items()}
            params = {("PanelJSON" if k.lower() == "paneljson" else "DetectedObjectsJSON" if k.lower() == "detectedobjectsjson"
                       else k): v for k, v in params.items()}
            if label == "Update Panel":
                self.update_panel(json.loads(params["PanelJSON"]))
            elif label == "Update Detected Objects":
                self.update_objects(json.loads(params["DetectedObjectsJSON"]))
            elif label == "Poll UIEvents":
                if self.on_poll:
                    self.on_poll(self)
                since = int(params.get("SinceSequence", 0))
                newer = [e for e in self.events if e["seq"] > since]
                return {"ReturnedValues": [{"LatestSequence": len(self.events),
                                            "Events": [json.dumps(e) for e in newer]}]}
            return {"ReturnedValues": [{}]}
        raise AssertionError(f"unexpected request {method} {endpoint}")


def operator(action):
    """An on_poll that does `action` once, after the tool has read its baseline (its first poll):
    a person taps after being asked, not before."""
    state = {"polls": 0}

    def on_poll(hud):
        state["polls"] += 1
        if state["polls"] == 2:
            action(hud)
    return on_poll


def listen_for_background_prompt(monkeypatch, hud):
    """Route the tool's prints through a check for its 'tap empty background' prompt, as a person reads it."""
    def hooked(*args, **kwargs):
        if hud.answers_background and any("tap empty background" in str(a) for a in args):
            hud.background_requested = True
        print(*args, **kwargs)
    monkeypatch.setattr(ubt, "print", hooked, raising=False)


@pytest.fixture
def fake_time(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(ubt, "_sleep", clock.sleep)
    monkeypatch.setattr(ubt, "_clock", clock.time)
    return clock


def _client(hud):
    client = BridgeClient()
    client._request = hud.request
    return client


def _push_and_check(hud, scenario):
    client = _client(hud)
    pan = PANEL_SCENARIOS[scenario][0]
    client.update_panel(pan)
    rows = read_panel_buttons(client, ROOT)
    return check_button_styles(pan["buttons"], rows), rows


def _statuses(results):
    return {what: status for status, what, _ in results}


# ── vocabulary ───────────────────────────────────────────────────────

def test_ui_button_style_matches_the_blueprint_switch():
    assert ui_button_style("unavailable") == "unavailable"
    assert ui_button_style("Unavailable") == "unavailable"  # the switch ignores case
    for fallback in (None, "", "normal", "warning", "bogus"):
        assert ui_button_style(fallback) == "valid", fallback


def test_the_styles_scenario_sends_every_style_once_and_labels_it():
    buttons = PANEL_SCENARIOS["styles"][0]["buttons"]
    sent = [b.get("style") for b in buttons]
    assert sent[0] is None  # absent
    assert sorted(s for s in sent if s) == sorted(ubt.BUTTON_STYLES)
    for b in buttons:
        assert b["label"] == (b.get("style") or "absent")


def test_unmangle_drops_blueprint_guid_suffixes():
    assert unmangle({f"button_id_2_{GUID}": "a", f"label_4_{GUID}": "b", "plain": 1}) == \
        {"button_id": "a", "label": "b", "plain": 1}


# ── finding the widgets ──────────────────────────────────────────────

def test_the_game_instance_is_found_in_one_batch():
    hud = FakeHUD()
    assert find_game_instance(_client(hud)) == ROOT


def test_no_hud_means_no_game_instance():
    hud = FakeHUD()
    del hud.objects[ARM_STATUS]
    assert find_game_instance(_client(hud)) is None


def test_only_the_buttons_still_on_the_panel_count():
    hud = FakeHUD()
    _, rows = _push_and_check(hud, "styles")
    assert [r["button_id"] for r in rows] == [b["button_id"] for b in PANEL_SCENARIOS["styles"][0]["buttons"]]


def test_buttons_numbered_past_the_first_chunks_are_still_found():
    hud = FakeHUD(start_k=2100)
    _, rows = _push_and_check(hud, "styles")
    assert len(rows) == len(PANEL_SCENARIOS["styles"][0]["buttons"])
    assert min(r["k"] for r in rows) > 2100


def test_buttons_the_panel_builds_a_read_later_are_waited_for(fake_time):
    hud = FakeHUD(build_late=True, cancel_wired=True)
    client = _client(hud)
    check, failures = ubt._reporter()
    rows = ubt.run_style_checks(client, ROOT, "styles", check)
    assert len(rows) == len(PANEL_SCENARIOS["styles"][0]["buttons"]) and not [f for f in failures if "once" in f]


def test_a_later_push_replaces_the_earlier_buttons():
    hud = FakeHUD()
    client = _client(hud)
    client.update_panel(PANEL_SCENARIOS["stress"][0])
    client.update_panel(PANEL_SCENARIOS["error"][0])
    assert [r["button_id"] for r in read_panel_buttons(client, ROOT)] == ["retry", "cancel"]


# ── style checks ─────────────────────────────────────────────────────

def test_on_todays_ui_the_unwired_cancel_fails_and_warning_is_flagged():
    results, _ = _push_and_check(FakeHUD(), "styles")
    failed = {what for status, what, _ in results if status == "FAIL"}
    assert failed == {"style:cancel shows its label 'cancel'", "invalid and cancel look different"}
    assert ("warn", "the contract's warning looks like normal") in {(s, w) for s, w, _ in results}


def test_a_ui_that_draws_every_style_as_meant_passes_cleanly():
    results, _ = _push_and_check(FakeHUD(cancel_wired=True, warning_distinct=True), "styles")
    assert all(status == "pass" for status, _, _ in results), [r for r in results if r[0] != "pass"]


def test_absent_and_normal_must_look_like_valid():
    statuses = _statuses(_push_and_check(FakeHUD(cancel_wired=True), "styles")[0])
    assert statuses["style:absent (style None) looks like valid"] == "pass"
    assert statuses["style:normal (style 'normal') looks like valid"] == "pass"


def test_two_styles_drawn_alike_fail():
    statuses = _statuses(_push_and_check(FakeHUD(cancel_wired=True, error_looks_valid=True), "styles")[0])
    assert statuses["valid and error look different"] == "FAIL"


def test_unavailable_differs_from_valid_by_its_label_opacity_alone():
    statuses = _statuses(_push_and_check(FakeHUD(cancel_wired=True), "styles")[0])
    assert statuses["valid and unavailable look different"] == "pass"


def test_a_button_on_the_panel_twice_fails():
    statuses = _statuses(_push_and_check(FakeHUD(cancel_wired=True, duplicate_live=True), "styles")[0])
    assert statuses["style:absent is on the panel once"] == "FAIL"


def test_unknown_empty_and_differently_cased_styles_fall_back_correctly():
    results, _ = _push_and_check(FakeHUD(), "style-fallback")
    assert all(status == "pass" for status, _, _ in results), [r for r in results if r[0] != "pass"]
    statuses = _statuses(results)
    for what in ("fallback:bogus (style 'bogus') looks like valid", "fallback:VALID (style 'VALID') looks like valid",
                 "fallback:empty (style '') looks like valid",
                 "fallback:Unavailable (style 'Unavailable') looks like unavailable",
                 "fallback:Error (style 'Error') looks like error"):
        assert statuses[what] == "pass", what


def test_a_case_sensitive_switch_is_caught():
    statuses = _statuses(_push_and_check(FakeHUD(case_sensitive=True), "style-fallback")[0])
    assert statuses["fallback:Unavailable (style 'Unavailable') looks like unavailable"] == "FAIL"


def test_styles_exits_nonzero_on_todays_ui_and_zero_once_cancel_is_wired(fake_time, capsys):
    args = type("A", (), {"scenario": "styles", "wait": False, "timeout": 1.0})()
    assert ubt.cmd_styles(_client(FakeHUD()), args) == 1
    assert "shows='OPEN'" in capsys.readouterr().out
    assert ubt.cmd_styles(_client(FakeHUD(cancel_wired=True, warning_distinct=True)), args) == 0


def test_styles_explains_when_there_is_no_hud(fake_time, capsys):
    hud = FakeHUD()
    del hud.objects[ARM_STATUS]
    args = type("A", (), {"scenario": "styles", "wait": False, "timeout": 1.0})()
    assert ubt.cmd_styles(_client(hud), args) == 1
    assert "no HUD found" in capsys.readouterr().err


# ── POIs ─────────────────────────────────────────────────────────────

def test_one_live_poi_per_object_and_none_left_over():
    hud = FakeHUD()
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene", variant="b"), "", ""))
    objects = sample_objects("scene") + sample_objects("scene", variant="unkeyed")
    client.update_detected_objects(ubt.detected_objects_payload(objects, "", ""))
    results = check_pois(objects, read_pois(client, ROOT))
    assert all(status == "pass" for status, _, _ in results), results


def test_pois_released_by_a_later_push_are_not_counted():
    hud = FakeHUD()
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene", variant="b"), "", ""))
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene"), "", ""))
    # set b's POIs still answer (released, Slot ''), so only the Slot filter keeps them out
    assert any(p["DetectedObjectData"]["InstanceId"] == "jug_5" for p in hud.objects.values()
               if "DetectedObjectData" in p)
    results = check_pois(sample_objects("scene"), read_pois(client, ROOT))
    assert all(status == "pass" for status, _, _ in results), results


def test_a_long_lived_poi_far_below_newer_ones_does_not_hide_them():
    hud = FakeHUD()
    client = _client(hud)
    keep = sample_objects("scene")[:1]
    client.update_detected_objects(ubt.detected_objects_payload(keep, "", ""))
    hud.next_poi_k = 3200  # thousands of tracker ids came and went, and were collected
    objects = keep + sample_objects("scene")[1:]
    client.update_detected_objects(ubt.detected_objects_payload(objects, "", ""))
    assert sorted(r["k"] for r in read_pois(client, ROOT)) == [0, 3200, 3201]


def test_an_extra_id_less_poi_or_a_doubled_one_fails():
    hud = FakeHUD(leave_released_pois=True)
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene", variant="unkeyed"), "", ""))
    one_unkeyed = sample_objects("scene") + sample_objects("scene", variant="unkeyed")[:1]
    client.update_detected_objects(ubt.detected_objects_payload(one_unkeyed, "", ""))
    assert _statuses(check_pois(one_unkeyed, read_pois(client, ROOT)))["1 POI(s) without an id"] == "FAIL"
    hud = FakeHUD(duplicate_poi=True)
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene"), "", ""))
    assert _statuses(check_pois(sample_objects("scene"), read_pois(client, ROOT)))["one POI for cup_1"] == "FAIL"


def test_a_tap_dims_id_less_pois_too():
    hud = FakeHUD()
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(
        sample_objects("scene") + sample_objects("scene", variant="unkeyed"), "", ""))
    hud.tap_poi("bowl_2")
    rows = read_pois(client, ROOT)
    assert [r["alpha"] for r in rows if not r["instance_id"]] == [0.4, 0.4]
    assert all(status == "pass" for status, _, _ in check_poi_dimming(rows, "bowl_2", 0.4))


def test_pois_dimmed_with_nothing_tapped_fail(fake_time, capsys):
    args = type("A", (), {"tap": False, "timeout": 1.0})()
    assert ubt.cmd_pois(_client(FakeHUD(dim_without_tap=True)), args) == 1
    assert "FAIL  POI cup_1 at opacity 1" in capsys.readouterr().out


def test_a_poi_left_over_from_an_earlier_push_fails():
    hud = FakeHUD(leave_released_pois=True)
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene", variant="b"), "", ""))
    two = sample_objects("scene")[:2]
    client.update_detected_objects(ubt.detected_objects_payload(two, "", ""))
    statuses = _statuses(check_pois(two, read_pois(client, ROOT)))
    assert statuses["no POIs left over from an earlier push"] == "FAIL"


def test_a_tap_dims_every_other_poi_to_the_layers_opacity():
    hud = FakeHUD()
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene"), "", ""))
    hud.tap_poi("bowl_2")
    rows = read_pois(client, ROOT)
    assert all(status == "pass" for status, _, _ in check_poi_dimming(rows, "bowl_2", 0.4))
    hud.tap_background()
    assert all(status == "pass" for status, _, _ in check_poi_dimming(read_pois(client, ROOT), None, 0.4))


def test_a_layer_that_does_not_dim_fails():
    hud = FakeHUD(poi_dims=False)
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene"), "", ""))
    hud.tap_poi("bowl_2")
    statuses = _statuses(check_poi_dimming(read_pois(client, ROOT), "bowl_2", 0.4))
    assert statuses["POI cup_1 at opacity 0.4"] == "FAIL"


def _tap_args(timeout=5.0):
    return type("A", (), {"tap": True, "timeout": timeout})()


def test_pois_tap_round_trip_passes(fake_time, capsys, monkeypatch):
    hud = FakeHUD()
    hud.answers_background = True
    listen_for_background_prompt(monkeypatch, hud)
    hud.on_poll = operator(lambda h: h.tap_poi("bowl_2"))
    assert ubt.cmd_pois(_client(hud), _tap_args()) == 0
    out = capsys.readouterr().out
    assert "[POI layer]" in out and "FAIL" not in out
    after_tap, after_background = out.split("tap empty background")
    assert "pass  POI cup_1 at opacity 0.4" in after_tap and "pass  POI bowl_2 at opacity 1" in after_tap
    assert "pass  POI cup_1 at opacity 1" in after_background


def test_pois_tap_on_a_layer_that_does_not_dim_fails(fake_time, capsys, monkeypatch):
    hud = FakeHUD(poi_dims=False)
    hud.answers_background = True
    listen_for_background_prompt(monkeypatch, hud)
    hud.on_poll = operator(lambda h: h.tap_poi("bowl_2"))
    assert ubt.cmd_pois(_client(hud), _tap_args()) == 1
    assert "FAIL  POI cup_1 at opacity 0.4" in capsys.readouterr().out


def test_pois_tap_without_the_background_tap_fails(fake_time, capsys):
    hud = FakeHUD()  # nobody taps the background
    hud.on_poll = operator(lambda h: h.tap_poi("bowl_2"))
    assert ubt.cmd_pois(_client(hud), _tap_args()) == 1
    assert "FAIL  POI cup_1 at opacity 1" in capsys.readouterr().out


def test_pois_tap_with_no_tap_at_all_fails_at_the_timeout(fake_time, capsys):
    assert ubt.cmd_pois(_client(FakeHUD()), _tap_args(timeout=2.0)) == 1
    assert "FAIL  a POI was tapped" in capsys.readouterr().out


def test_pois_tap_without_a_readable_dim_opacity_fails_instead_of_crashing(fake_time, capsys):
    hud = FakeHUD()
    del hud.objects[LAYER]["UnselectedPOIOpacity"]
    assert ubt.cmd_pois(_client(hud), _tap_args()) == 1
    assert "UnselectedPOIOpacity is readable" in capsys.readouterr().out


def test_pois_tap_on_a_camera_box_is_not_a_poi_tap(fake_time, capsys, monkeypatch):
    hud = FakeHUD()
    hud.answers_background = True
    listen_for_background_prompt(monkeypatch, hud)
    def tap_the_camera_box(h):
        h.last_tapped = "bowl_2"
        h._apply_opacity()
        h.post({"event_type": "object_tapped", "instance_id": "bowl_2", "stream_id": "camera/scene/color",
                "tap_point": [600.0, 300.0], "image_size": [1280, 720]})
    hud.on_poll = operator(tap_the_camera_box)
    args = type("A", (), {"tap": True, "timeout": 5.0})()
    assert ubt.cmd_pois(_client(hud), args) == 1
    assert "came from the camera image" in capsys.readouterr().out


def test_an_earlier_tap_still_recorded_is_a_note_not_a_failure(fake_time, capsys):
    hud = FakeHUD()
    client = _client(hud)
    client.update_detected_objects(ubt.detected_objects_payload(sample_objects("scene"), "", ""))
    hud.tap_poi("cup_1")
    args = type("A", (), {"tap": False, "timeout": 1.0})()
    assert ubt.cmd_pois(client, args) == 0
    assert "still the last tapped POI" in capsys.readouterr().out


def test_a_collapsed_layer_is_explained(fake_time, capsys):
    hud = FakeHUD()
    hud.objects[LAYER]["Visibility"] = "Collapsed"
    args = type("A", (), {"tap": False, "timeout": 1.0})()
    ubt.cmd_pois(_client(hud), args)
    assert "press the camera toggle" in capsys.readouterr().out.lower()


# ── events ───────────────────────────────────────────────────────────

def test_tap_origin_is_told_by_what_an_object_tap_carries():
    assert tap_origin({"instance_id": "a", "stream_id": "camera/scene/color", "tap_point": [1, 2]}) == "camera image"
    # a marker tap with no frame size yet: no point, but still its camera's stream id
    assert tap_origin({"instance_id": "a", "stream_id": "camera/scene/color"}) == "camera image"
    assert tap_origin({"instance_id": "a", "stream_id": "engine/overlay"}) == "camera image"
    assert tap_origin({"instance_id": "a", "stream_id": "scene"}) == "3D view"
    assert tap_origin({"instance_id": "a"}) == "POI layer"


def test_a_poi_tap_is_described_as_one():
    assert ubt._describe({"event_type": "object_tapped", "instance_id": "cup_1"}) == "object_tapped 'cup_1' [POI layer]"


# ── selftest ─────────────────────────────────────────────────────────

def test_selftest_checks_styles_and_pois_and_fails_on_the_unwired_cancel(fake_time, capsys):
    args = type("A", (), {})()
    assert ubt.cmd_selftest(_client(FakeHUD()), args) == 1
    out = capsys.readouterr().out
    assert "FAIL  style:cancel shows its label 'cancel'" in out
    assert "pass  fallback:bogus (style 'bogus') looks like valid" in out   # style-fallback ran
    assert "pass  one POI for cup_1" in out                                 # the POI checks ran


def test_a_remote_control_error_in_the_hud_checks_does_not_end_selftest(fake_time, capsys):
    hud = FakeHUD(cancel_wired=True, warning_distinct=True)
    real = hud.request

    def failing(method, endpoint, payload=None):
        if endpoint == "/remote/batch" and any(r["Body"]["propertyName"] == "PanelButton" for r in payload["Requests"]):
            raise BridgeError("PUT /remote/batch failed: timed out")
        return real(method, endpoint, payload)
    hud.request = failing
    args = type("A", (), {})()
    assert ubt.cmd_selftest(_client(hud), args) == 1
    out = capsys.readouterr().out
    assert "FAIL  HUD readback" in out
    assert "joints" in out and out.rstrip().splitlines()[-1].startswith("FAILED")   # carried on to the end


def test_selftest_passes_on_a_ui_that_draws_its_styles(fake_time, capsys):
    args = type("A", (), {})()
    assert ubt.cmd_selftest(_client(FakeHUD(cancel_wired=True, warning_distinct=True)), args) == 0


# ── transport ────────────────────────────────────────────────────────

def test_a_raw_newline_in_an_error_message_does_not_empty_a_batch(monkeypatch):
    body = ('{"Responses": [{"RequestId": 0, "ResponseCode": 400, "ResponseBody": {"errorMessage": "line one\n'
            'line two"}}, {"RequestId": 1, "ResponseCode": 200, "ResponseBody": {"Text": "ok"}}]}')

    class Response:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def read(self): return body.encode()

    monkeypatch.setattr(ubt.urllib.request, "urlopen", lambda req, timeout=None: Response())
    found = BridgeClient().read_many([("/a", "Text"), ("/b", "Text")])
    assert found == {1: "ok"}


def test_the_walkthrough_poi_step_without_taps_checks_the_pois_and_does_not_wait(fake_time, capsys):
    args = type("A", (), {"no_taps": True, "timeout": 60.0})()
    ubt._walkthrough_poi_tap(_client(FakeHUD()), args)
    out = capsys.readouterr().out
    assert "pass  one POI for cup_1" in out and "tap one of the POIs" not in out
