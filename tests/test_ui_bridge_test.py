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
    BridgeClient,
    BridgeError,
    button,
    detected_object,
    detected_objects_payload,
    panel,
    sample_objects,
    scenario_buttons,
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
    a box drawn for 1280x720 lands off-screen. Coordinates <= 1.0 are used as-is."""
    for obj in sample_objects("scene"):
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
    for pixels in (False, True):
        for obj in sample_objects("scene", pixels=pixels):
            assert obj["box"]["X"] != obj["box"]["Z"], obj
            assert obj["box"]["Y"] != obj["box"]["W"], obj


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
