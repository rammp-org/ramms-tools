"""Offline tests for ramms_tools.scripts.ui_bridge_test.

Everything here runs without an Unreal editor. What's worth pinning down is
the wire shape, because both of these failure modes are silent against a real
UE: a bare JSON array into an FVector4 property is rejected outright, and a
mis-named function parameter is accepted with HTTP 200 and arrives as an empty
string. Neither surfaces as an error the caller can see.
"""
from __future__ import annotations

import json

import pytest

from ramms_tools.scripts.ui_bridge_test import (
    PANEL_SCENARIOS,
    SAMPLE_SETS,
    BridgeClient,
    BridgeError,
    button,
    detected_object,
    detected_objects_payload,
    panel,
    sample_objects,
    scenario_buttons,
    _positive_int,
)


# ── panels ───────────────────────────────────────────────────────────

def test_panel_always_carries_a_buttons_array():
    """Even a pure status panel has `buttons`, so UE can iterate unconditionally."""
    assert panel()["buttons"] == []
    assert panel(message="hi")["buttons"] == []


def test_optional_panel_keys_are_omitted_not_nulled():
    """UE-side code uses key-existence checks, so None must not be serialised."""
    bare = panel(title="t", message="m")
    assert "progress" not in bare
    assert "style" not in bare

    full = panel(progress={"phase": "p", "pct": 0.5, "message": "m", "pausable": True},
                 style="error")
    assert full["progress"]["pct"] == 0.5
    assert full["style"] == "error"


def test_button_style_is_omitted_when_unset():
    """An absent per-button style means 'normal'; sending null would not."""
    assert button("id", "Label") == {"button_id": "id", "label": "Label"}
    assert button("id", "Label", "warning")["style"] == "warning"


@pytest.mark.parametrize("name", sorted(PANEL_SCENARIOS))
def test_every_scenario_is_serialisable_and_well_formed(name):
    pan, expectation = PANEL_SCENARIOS[name]
    round_tripped = json.loads(json.dumps(pan))
    assert set(round_tripped) <= {"title", "message", "buttons", "progress", "style"}
    assert isinstance(round_tripped["buttons"], list)
    for btn in round_tripped["buttons"]:
        assert btn["button_id"] and btn["label"]
        assert btn.get("style", "normal") in ("normal", "warning", "error")
    assert expectation, "each scenario documents what the operator should see"


def test_scenario_button_ids_are_unique_within_a_panel():
    for name in PANEL_SCENARIOS:
        ids = scenario_buttons(name)
        assert len(ids) == len(set(ids)), name


# ── detected objects ─────────────────────────────────────────────────

def test_box_and_centroid_are_objects_not_arrays():
    """UE's FJsonObjectConverter rejects a bare array into FVector4/FVector2D."""
    obj = detected_object("cup_1", "cup", (10, 20, 30, 40))
    assert obj["box"] == {"X": 10, "Y": 20, "Z": 30, "W": 40}
    assert isinstance(obj["centroid"], dict)
    assert set(obj["centroid"]) == {"X", "Y"}


def test_centroid_defaults_to_the_box_centre():
    obj = detected_object("cup_1", "cup", (10, 20, 30, 40))
    assert obj["centroid"] == {"X": 20.0, "Y": 30.0}


def test_explicit_centroid_wins():
    obj = detected_object("tap_3", "faucet", (0, 0, 100, 100), centroid=(90, 10))
    assert obj["centroid"] == {"X": 90, "Y": 10}


def test_samples_are_normalized_by_default():
    """Pixel boxes get divided by the live frame size (or a 640x480 fallback), so
    a box drawn for 1280x720 lands off-screen. Coordinates <= 1.0 are used as-is.

    Every set, not just "a": a set that strays above 1.0 is silently rescaled by the
    widget and lands somewhere else entirely."""
    for variant in SAMPLE_SETS:
        for obj in sample_objects("scene", variant=variant):
            for key in ("X", "Y", "Z", "W"):
                assert 0.0 <= obj["box"][key] <= 1.0, obj
            for key in ("X", "Y"):
                assert 0.0 <= obj["centroid"][key] <= 1.0, obj


def test_pixel_samples_fit_a_1280x720_frame():
    for obj in sample_objects("scene", pixels=True):
        assert obj["box"]["Z"] <= 1280 and obj["box"]["W"] <= 720, obj
        assert obj["box"]["X"] >= 0 and obj["box"]["Y"] >= 0, obj
        # Above 1.0 everywhere, or the widget would read them as normalized.
        assert obj["box"]["Z"] > 1.0 and obj["box"]["W"] > 1.0, obj


def test_box_is_never_degenerate():
    """A box whose min equals its max reads as 'no box' and is skipped."""
    for variant in SAMPLE_SETS:
        for obj in sample_objects("scene", variant=variant):
            assert obj["box"]["X"] != obj["box"]["Z"], obj
            assert obj["box"]["Y"] != obj["box"]["W"], obj
    for obj in sample_objects("scene", pixels=True):
        assert obj["box"]["X"] != obj["box"]["Z"], obj
        assert obj["box"]["Y"] != obj["box"]["W"], obj


def test_centroid_is_never_the_skipped_origin():
    """A centroid of exactly (0, 0) with no box is the orchestrator's 'no position',
    and is dropped. A sample that hit it would silently never appear."""
    for variant in SAMPLE_SETS:
        for obj in sample_objects("scene", variant=variant):
            assert (obj["centroid"]["X"], obj["centroid"]["Y"]) != (0.0, 0.0), obj


def test_sets_a_and_b_share_no_instance_id():
    """The 'Objects replaced' step proves removal by pushing a disjoint set. A shared
    id would be reused rather than removed, and the step would pass either way."""
    a = {o["instance_id"] for o in sample_objects("scene", variant="a")}
    b = {o["instance_id"] for o in sample_objects("scene", variant="b")}
    assert a and b and not (a & b), (a, b)


def test_unkeyed_set_has_no_instance_ids():
    objs = sample_objects("scene", variant="unkeyed")
    assert objs and all(o["instance_id"] == "" for o in objs), objs


def test_count_takes_the_first_n():
    objs = sample_objects("scene", variant="b", count=1)
    assert [o["instance_id"] for o in objs] == ["jug_5"], objs


def test_pixels_with_another_set_is_refused():
    """Silently returning set 'a' would make a programmatic caller test the wrong set."""
    with pytest.raises(ValueError):
        sample_objects("scene", pixels=True, variant="b")
    with pytest.raises(KeyError):
        sample_objects("scene", variant="nope")


def test_count_must_be_at_least_one():
    """0 would be a clear wearing a push's message; a negative value drops from the END."""
    import argparse

    assert _positive_int("3") == 3
    for bad in ("0", "-1", "x"):
        with pytest.raises(argparse.ArgumentTypeError):
            _positive_int(bad)


def test_clear_payload_carries_an_empty_stream_id():
    """What the real bridge sends, and what the final walkthrough step asserts."""
    assert detected_objects_payload([], "", "") == {
        "stream_id": "",
        "selected_instance_id": "",
        "objects": [],
    }


def test_payload_carries_selection_and_stream_id():
    payload = detected_objects_payload(sample_objects("scene"), "cup_1", "camera/scene/color")
    assert payload["selected_instance_id"] == "cup_1"
    assert payload["stream_id"] == "camera/scene/color"
    assert [o["instance_id"] for o in payload["objects"]] == ["cup_1", "bowl_2", "tap_3"]
    assert {o["source"] for o in payload["objects"]} == {"scene"}


def test_clearing_is_an_empty_list_not_a_missing_key():
    payload = detected_objects_payload([], "", "")
    assert payload["objects"] == []
    assert "objects" in payload


# ── the parameter-name guard ─────────────────────────────────────────

def _client_with_preset(arguments):
    client = BridgeClient()
    client._labels = {name: name for name in arguments}
    client._arguments = dict(arguments)
    return client


def test_unknown_parameter_is_refused_before_it_reaches_ue():
    """An unrecognised name must fail loudly here.

    UE accepts the call with HTTP 200 and hands the function an empty string --
    for UpdatePanel that clears the panel rather than updating it, with nothing
    reported anywhere.
    """
    client = _client_with_preset({"UpdatePanel": ["PanelJSON"]})
    with pytest.raises(BridgeError) as excinfo:
        client.call("UpdatePanel", NotAParameter="{}")
    message = str(excinfo.value)
    assert "NotAParameter" in message
    assert "PanelJSON" in message


@pytest.mark.parametrize("spelling", ["PanelJSON", "PanelJson", "paneljson"])
def test_parameter_matching_is_case_insensitive(monkeypatch, spelling):
    """UE resolves parameters case-insensitively, verified against a live editor:
    all three of these deliver the payload. Refusing them would block calls that
    work."""
    client = _client_with_preset({"UpdatePanel": ["PanelJSON"]})
    sent = {}
    monkeypatch.setattr(client, "_request",
                        lambda m, e, payload=None: sent.update(payload=payload)
                        or {"ReturnedValues": [{}]})
    client.call("UpdatePanel", **{spelling: "{}"})
    assert sent["payload"]["Parameters"] == {spelling: "{}"}


def test_declared_parameter_is_not_refused(monkeypatch):
    client = _client_with_preset({"UpdatePanel": ["PanelJSON"]})
    sent = {}

    def fake_request(method, endpoint, payload=None):
        sent["endpoint"] = endpoint
        sent["payload"] = payload
        return {"ReturnedValues": [{}]}

    monkeypatch.setattr(client, "_request", fake_request)
    client.call("UpdatePanel", PanelJSON="{}")
    assert sent["payload"]["Parameters"] == {"PanelJSON": "{}"}


def test_function_label_is_url_quoted(monkeypatch):
    """The endpoint takes the exposed label ('Poll UIEvents'), not the C++ name."""
    client = BridgeClient()
    client._labels = {"PollUIEvents": "Poll UIEvents"}
    client._arguments = {"PollUIEvents": ["SinceSequence"]}
    seen = {}

    def fake_request(method, endpoint, payload=None):
        seen["endpoint"] = endpoint
        return {"ReturnedValues": [{"LatestSequence": 3, "Events": []}]}

    monkeypatch.setattr(client, "_request", fake_request)
    client.poll_events(0)
    assert seen["endpoint"].endswith("/function/Poll%20UIEvents")


# ── event parsing ────────────────────────────────────────────────────

def test_poll_events_parses_and_survives_a_bad_event(monkeypatch, capsys):
    client = BridgeClient()
    client._labels = {"PollUIEvents": "Poll UIEvents"}
    client._arguments = {"PollUIEvents": ["SinceSequence"]}

    monkeypatch.setattr(client, "_request", lambda *a, **k: {"ReturnedValues": [{
        "LatestSequence": 2,
        "Events": ['{"event_type":"button_press","button_id":"ok","seq":1}',
                   "not json at all",
                   '{"event_type":"object_tapped","instance_id":"cup_1","seq":2}'],
    }]})

    latest, events = client.poll_events(0)
    assert latest == 2
    assert [e["event_type"] for e in events] == ["button_press", "object_tapped"]
    assert "unparseable" in capsys.readouterr().out


def test_remote_control_errors_are_raised_not_returned():
    """Remote Control reports failures in-band with HTTP 200."""
    client = BridgeClient()

    class FakeResponse:
        def read(self):
            return b'{ "errorMessage": "Unable to resolve the preset field." }'

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    import ramms_tools.scripts.ui_bridge_test as module

    original = module.urllib.request.urlopen
    module.urllib.request.urlopen = lambda *a, **k: FakeResponse()
    try:
        with pytest.raises(BridgeError, match="preset field"):
            client._request("GET", "/remote/preset/RCP_RammsBridge")
    finally:
        module.urllib.request.urlopen = original


# ── waiting for taps ─────────────────────────────────────────────────

class _FakeClient:
    """Minimal stand-in that records which baselines were polled."""

    def __init__(self, events_by_since=None, latest=0):
        self.events_by_since = events_by_since or {}
        self.latest = latest
        self.polled = []

    def poll_events(self, since=0):
        self.polled.append(since)
        return self.latest, self.events_by_since.get(since, [])


def test_await_uses_a_supplied_baseline_without_rebaselining():
    """A tap landing between the push and the wait must still be seen.

    Re-polling from 0 here to find "current" would move the cursor past that
    tap and time out a wait that actually succeeded.
    """
    import ramms_tools.scripts.ui_bridge_test as module

    tap = {"event_type": "object_tapped", "instance_id": "cup_1", "seq": 6}
    client = _FakeClient({5: [tap]}, latest=6)
    assert module._await(client, {"cup_1"}, 5.0, "object_tapped", since=5) == 0
    assert 0 not in client.polled, "must not re-baseline when given a sequence"
    assert client.polled[0] == 5


def test_await_without_a_baseline_starts_from_current():
    import ramms_tools.scripts.ui_bridge_test as module

    client = _FakeClient({}, latest=9)
    module._await(client, {"nope"}, 0.01, "button_press")
    assert client.polled[0] == 0, "first poll establishes the baseline"


def test_await_ignores_a_stale_tap_from_before_the_baseline():
    import ramms_tools.scripts.ui_bridge_test as module

    stale = {"event_type": "object_tapped", "instance_id": "cup_1", "seq": 2}
    client = _FakeClient({7: []}, latest=7)
    client.events_by_since[2] = [stale]
    assert module._await(client, {"cup_1"}, 0.05, "object_tapped", since=7) == 1


# ── exit statuses ────────────────────────────────────────────────────

def test_interrupting_an_expectation_exits_nonzero():
    """--expect promises 'exit 0 only once this id arrives'.

    Cancelling the wait leaves it unmet, and a script must not read that as a
    successful round-trip.
    """
    import argparse

    import ramms_tools.scripts.ui_bridge_test as module

    class Interrupting:
        def poll_events(self, since=0):
            raise KeyboardInterrupt

    args = argparse.Namespace(since=0, timeout=0.0, follow=True, interval=0.01,
                              expect="probe:ok")
    assert module.cmd_events(Interrupting(), args) == 1


def test_interrupting_a_watch_only_session_exits_zero():
    import argparse

    import ramms_tools.scripts.ui_bridge_test as module

    class Interrupting:
        def poll_events(self, since=0):
            raise KeyboardInterrupt

    args = argparse.Namespace(since=0, timeout=0.0, follow=True, interval=0.01,
                              expect="")
    assert module.cmd_events(Interrupting(), args) == 0


def test_stream_sends_synchronously_so_failures_surface(monkeypatch):
    """A diagnostic must not print 'sent N frames' over a failed send.

    With the default queued sender a socket error is logged by the worker while
    the command still reports success and exits 0.
    """
    import argparse

    import ramms_tools.streaming as streaming

    recorded = {}

    class RecordingSender:
        def __init__(self, host, port, queue_size=4):
            recorded["queue_size"] = queue_size
            self.dropped_frames = 0

        def connect(self):
            pass

        def send_image(self, *a, **k):
            pass

        def send_depth_uint16(self, *a, **k):
            pass

        def disconnect(self, flush=False):
            pass

    monkeypatch.setattr(streaming, "StreamSender", RecordingSender)
    client = _FakeClient({}, latest=0)
    client.get_setting = lambda name, *a, **k: ""
    args = argparse.Namespace(host="127.0.0.1", stream_port=30030, seconds=0.0,
                              fps=60.0, width=4, height=2, overlay=False)
    import ramms_tools.scripts.ui_bridge_test as module

    assert module.cmd_stream(client, args) == 0
    assert recorded["queue_size"] == 0


# ── binding checks ───────────────────────────────────────────────────

def _preset_json(entries):
    """entries: [(cpp_name, label, [owner_paths])]"""
    return {"Preset": {"Groups": [{
        "Name": "All",
        "ExposedProperties": [],
        "ExposedFunctions": [
            {"DisplayName": label,
             "UnderlyingFunction": {"Name": name,
                                    "Arguments": [{"Name": "PanelJSON"}]},
             "OwnerObjects": [{"Path": p} for p in owners]}
            for name, label, owners in entries
        ],
    }]}}


def _client_with_preset_json(payload, monkeypatch, name="RCPS", presets=None):
    """A client wired to a fake editor serving one preset.

    Routes /remote/presets and /remote/preset/<name> separately, as the real
    editor does, so preset discovery is exercised rather than bypassed.
    """
    listing = {"Presets": presets if presets is not None else [{"Name": name}]}
    calls = []

    def fake_request(method, endpoint, p=None):
        calls.append((method, endpoint))
        if method == "GET" and endpoint == "/remote/presets":
            return listing
        if method == "GET":
            return payload(endpoint) if callable(payload) else payload
        return {"ReturnedValues": [{}]}

    client = BridgeClient()
    client.preset = name  # pinned: these cases are about bindings, not discovery
    monkeypatch.setattr(client, "_request", fake_request)
    client.calls = calls
    return client


def test_unbound_function_is_refused_rather_than_silently_succeeding(monkeypatch):
    """Remote Control answers HTTP 200 for a call that reaches no object.

    That is how a moved level turns every push into a no-op that reports
    success, so the client has to catch it rather than trust the status.
    """
    client = _client_with_preset_json(
        _preset_json([("UpdatePanel", "Update Panel", [])]), monkeypatch)
    with pytest.raises(BridgeError) as excinfo:
        client.call("UpdatePanel", PanelJSON="{}")
    message = str(excinfo.value)
    assert "bound to no object" in message
    assert "Rebind" in message


def test_bound_function_is_allowed(monkeypatch):
    client = _client_with_preset_json(
        _preset_json([("UpdatePanel", "Update Panel", ["/Game/Maps/M.M:X.Recv_1"])]),
        monkeypatch)
    assert client.call("UpdatePanel", PanelJSON="{}") == {}


def test_unbound_functions_lists_every_orphan(monkeypatch):
    client = _client_with_preset_json(_preset_json([
        ("UpdatePanel", "Update Panel", []),
        ("PollUIEvents", "Poll UIEvents", []),
        ("OpenCameraFeed", "Open Camera Feed", ["/Game/Maps/M.M:X.Recv_1"]),
    ]), monkeypatch)
    assert client.unbound_functions() == ["PollUIEvents", "UpdatePanel"]


def test_a_duplicate_exposure_prefers_the_bound_entry(monkeypatch):
    """Re-exposing after a level move leaves the stale entry and adds "Name (1)".

    Last-wins would pick whichever was serialised last, which may be the dead
    one; the bound entry is the only one worth calling.
    """
    client = _client_with_preset_json(_preset_json([
        ("UpdatePanel", "Update Panel", []),                       # stale
        ("UpdatePanel", "Update Panel (1)", ["/Game/M.M:X.Recv_1"]),  # live
    ]), monkeypatch)
    client.load_preset()
    assert client.label_for("UpdatePanel") == "Update Panel (1)"
    assert client._duplicates["UpdatePanel"] == ["Update Panel", "Update Panel (1)"]
    assert client.unbound_functions() == []


def test_a_duplicate_in_the_other_order_still_prefers_the_bound_entry(monkeypatch):
    """All three dicts must follow the same entry.

    Assigning _owners/_arguments before the preference guard is a plausible
    refactor that leaves the label on the live entry and the owners on the dead
    one, which reports a working function as orphaned and refuses every call.
    """
    live = {"DisplayName": "Update Panel (1)",
            "UnderlyingFunction": {"Name": "UpdatePanel",
                                   "Arguments": [{"Name": "PanelJSON"}]},
            "OwnerObjects": [{"Path": "/Game/M.M:X.Recv_1"}]}
    dead = {"DisplayName": "Update Panel",
            "UnderlyingFunction": {"Name": "UpdatePanel",
                                   "Arguments": [{"Name": "StaleArgument"}]},
            "OwnerObjects": []}
    client = _client_with_preset_json(
        {"Preset": {"Groups": [{"ExposedFunctions": [live, dead]}]}}, monkeypatch)
    client.load_preset()
    assert client.label_for("UpdatePanel") == "Update Panel (1)"
    assert client._arguments["UpdatePanel"] == ["PanelJSON"], "arguments follow the label"
    assert client._owners["UpdatePanel"] == ["/Game/M.M:X.Recv_1"]
    assert client.unbound_functions() == [], "the bound entry must not read as orphaned"


def test_binding_check_does_not_block_when_the_preset_is_unreadable(monkeypatch):
    """No preset data is not evidence of a broken binding."""
    client = BridgeClient()

    def fake_request(method, endpoint, payload=None):
        if method == "GET":
            raise BridgeError("preset unreadable")
        return {"ReturnedValues": [{}]}

    monkeypatch.setattr(client, "_request", fake_request)
    assert client.call("UpdatePanel", PanelJSON="{}") == {}


# ── preset discovery ─────────────────────────────────────────────────

def _full_preset(name, bound=True, owner="/Game/Maps/M.M:PersistentLevel.Recv_1"):
    owners = [owner] if bound else []
    return _preset_json([(f, f.replace("Update", "Update "), owners)
                         for f in ("UpdatePanel", "UpdateDetectedObjects", "PollUIEvents")])


def test_discovery_picks_the_preset_whose_bindings_are_intact(monkeypatch):
    """Presence is not enough: an orphaned entry is still listed and still
    answers HTTP 200 while reaching nothing."""
    def payload(endpoint):
        return _full_preset("x", bound=endpoint.endswith("Good"))

    client = _client_with_preset_json(
        payload, monkeypatch, presets=[{"Name": "Dead"}, {"Name": "Good"}])
    client.preset = "auto"
    assert client.discover_preset() == "Good"


def test_discovery_reports_why_each_candidate_was_rejected(monkeypatch):
    client = _client_with_preset_json(
        lambda endpoint: _full_preset("x", bound=False), monkeypatch,
        presets=[{"Name": "Orphaned"}])
    client.preset = "auto"
    with pytest.raises(BridgeError) as excinfo:
        client.discover_preset()
    message = str(excinfo.value)
    assert "Orphaned" in message and "bound to nothing" in message
    assert "--preset" in message


def test_discovery_rejects_a_preset_missing_the_api(monkeypatch):
    client = _client_with_preset_json(
        lambda endpoint: _preset_json([("UpdatePanel", "Update Panel", ["/x"])]),
        monkeypatch, presets=[{"Name": "Partial"}])
    client.preset = "auto"
    with pytest.raises(BridgeError, match="missing"):
        client.discover_preset()


def test_a_failed_preset_load_is_not_retried_on_every_call(monkeypatch):
    """One unreachable editor should not cost three full timeouts per command."""
    client = BridgeClient()
    attempts = []

    def fake_request(method, endpoint, p=None):
        attempts.append(endpoint)
        raise BridgeError("editor is not answering")

    monkeypatch.setattr(client, "_request", fake_request)
    for _ in range(3):
        with pytest.raises(BridgeError):
            client.load_preset()
    assert len(attempts) == 1, f"retried {len(attempts)} times: {attempts}"


def test_load_preset_is_idempotent(monkeypatch):
    """Calling it twice must not invent a duplicate that discover would report."""
    client = _client_with_preset_json(_full_preset("x"), monkeypatch)
    client.load_preset()
    client.load_preset()
    assert client._duplicates["UpdatePanel"] == ["Update Panel"]


def test_a_missing_owner_objects_key_is_unknown_not_orphaned(monkeypatch):
    """Only an empty list means orphaned; an absent key means the preset did
    not say, and blocking every call on that would be a false positive."""
    payload = {"Preset": {"Groups": [{"ExposedFunctions": [
        {"DisplayName": "Update Panel",
         "UnderlyingFunction": {"Name": "UpdatePanel",
                                "Arguments": [{"Name": "PanelJSON"}]}},
    ]}]}}
    client = _client_with_preset_json(payload, monkeypatch)
    client.load_preset()
    assert client.owners_for("UpdatePanel") is None
    assert client.unbound_functions() == []
    client.call("UpdatePanel", PanelJSON="{}")  # must not raise


def test_unbound_functions_returns_none_when_it_cannot_tell(monkeypatch):
    """[] is an all-clear; a failed lookup must not look like one."""
    client = BridgeClient()
    monkeypatch.setattr(client, "_request",
                        lambda *a, **k: (_ for _ in ()).throw(BridgeError("nope")))
    assert client.unbound_functions() is None


def test_a_non_dict_response_raises_bridge_error_not_attribute_error(monkeypatch):
    client = BridgeClient()
    client.preset = "RCPS"
    monkeypatch.setattr(client, "_request", lambda *a, **k: None)
    with pytest.raises(BridgeError):
        client.load_preset()


# ── frame pacing ─────────────────────────────────────────────────────

def test_each_channel_is_sent_once_per_frame_with_one_sleep_each():
    """The old loop slept three times but sent twice without --overlay, so the
    stream ran at two thirds of the rate asked for."""
    import ramms_tools.scripts.ui_bridge_test as module

    for channel_count in (2, 3):
        sends = [lambda i=i: order.append(i) for i in range(channel_count)]
        order, slept = [], []
        module._send_frame_spaced(sends, 0.0, 1.0, lambda d: slept.append(d))
        assert order == list(range(channel_count))
        assert len(slept) == channel_count, "one sleep per send, no more"
        assert slept == [(i + 1) * (1.0 / channel_count) for i in range(channel_count)]


def test_frame_deadlines_do_not_drift_with_the_cost_of_sending():
    """Deadlines are absolute, so time spent building a frame is absorbed
    rather than added to every interval."""
    import ramms_tools.scripts.ui_bridge_test as module

    slept = []
    for frame in range(3):
        module._send_frame_spaced([lambda: None, lambda: None],
                                  frame * 1.0, 1.0, lambda d: slept.append(d))
    assert slept == [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


def test_a_stop_between_channels_abandons_the_rest_of_the_frame():
    import ramms_tools.scripts.ui_bridge_test as module

    sent = []
    stopped = {"now": False}

    def send(i):
        sent.append(i)
        stopped["now"] = True

    sends = [lambda i=i: send(i) for i in range(3)]
    assert module._send_frame_spaced(sends, 0.0, 1.0, lambda d: None,
                                     stop=lambda: stopped["now"]) is False
    assert sent == [0], "must not keep sending ~900 KB per channel after a stop"
