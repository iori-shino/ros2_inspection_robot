"""Regression tests for geometry and message contracts; no ROS needed."""

import json
import math

import pytest

from inspection_robot.message_utils import decode_message, encode_message
from inspection_robot.patrol_logic import compute_target_error


def _message(kind, **changes):
    data = {"run_id": "test-run", "point_id": "B", "timestamp": 1.0}
    if kind == "status":
        data.update(
            state="INSPECTING", x=2.0, y=8.0, theta=0.0, detail="到达巡检点"
        )
    else:
        data.update(temperature_c=85.0, threshold_c=80.0, is_abnormal=True)
    data.update(changes)
    return data


def _assert_rejected(data, kind):
    # Validate both outgoing data and independently supplied incoming JSON.
    with pytest.raises(ValueError):
        encode_message(data, kind)
    with pytest.raises(ValueError):
        decode_message(json.dumps(data), kind)


@pytest.mark.parametrize(
    "pose,target,expected_distance,expected_heading",
    [
        ((0.0, 0.0, 0.0), (3.0, 4.0), 5.0, math.atan2(4.0, 3.0)),
        ((0.0, 0.0, 0.0), (0.0, 1.0), 1.0, math.pi / 2),
        ((0.0, 0.0, math.pi / 2), (1.0, 0.0), 1.0, -math.pi / 2),
        (
            (0.0, 0.0, math.radians(179)),
            (math.cos(math.radians(-179)), math.sin(math.radians(-179))),
            1.0, math.radians(2),
        ),
        (
            (0.0, 0.0, math.radians(-179)),
            (math.cos(math.radians(179)), math.sin(math.radians(179))),
            1.0, math.radians(-2),
        ),
        ((2.0, 8.0, 1.2), (2.0, 8.0), 0.0, 0.0),
    ],
    ids=["distance", "left", "right", "wrap-left", "wrap-right", "arrived"],
)
def test_geometry(pose, target, expected_distance, expected_heading):
    distance, heading = compute_target_error(*pose, *target)
    assert distance == pytest.approx(expected_distance, abs=1e-12)
    assert heading == pytest.approx(expected_heading, abs=1e-12)


@pytest.mark.parametrize(
    "state", ["WAITING", "MOVING", "INSPECTING", "COMPLETED", "ERROR"]
)
def test_status_round_trip(state):
    data = _message("status", state=state)
    text = encode_message(data, "status")
    assert json.loads(text) == data
    assert decode_message(text, "status") == data


@pytest.mark.parametrize("state", ["WAITING", "ERROR"])
def test_unknown_pose_allowed_while_waiting_or_failed(state):
    data = _message("status", state=state, x=None, y=None, theta=None)
    assert decode_message(encode_message(data, "status"), "status") == data


@pytest.mark.parametrize(
    "temperature,abnormal", [(79.0, False), (80.0, False), (85.0, True)]
)
def test_temperature_boundary(temperature, abnormal):
    data = _message("result", temperature_c=temperature, is_abnormal=abnormal)
    assert decode_message(encode_message(data, "result"), "result") == data
    _assert_rejected(dict(data, is_abnormal=not abnormal), "result")


@pytest.mark.parametrize("kind", ["status", "result"])
@pytest.mark.parametrize("change", ["missing", "extra"])
def test_exact_message_fields_required(kind, change):
    data = _message(kind)
    if change == "missing":
        del data["run_id"]
    else:
        data["unexpected"] = "value"
    _assert_rejected(data, kind)


@pytest.mark.parametrize("kind", ["status", "result"])
@pytest.mark.parametrize(
    "changes",
    [
        {"run_id": ""},
        {"point_id": "  "},
        {"run_id": 123},
        {"timestamp": -1.0},
        {"timestamp": True},
        {"timestamp": float("nan")},
        {"timestamp": float("inf")},
    ],
)
def test_invalid_identity_or_timestamp(kind, changes):
    _assert_rejected(_message(kind, **changes), kind)


@pytest.mark.parametrize(
    "changes",
    [
        {"state": "UNKNOWN"},
        {"x": None},
        {"x": None, "y": None, "theta": None},
        {"x": float("inf")},
        {"theta": True},
        {"detail": 123},
    ],
)
def test_invalid_status(changes):
    _assert_rejected(_message("status", **changes), "status")


@pytest.mark.parametrize(
    "changes",
    [
        {"temperature_c": True},
        {"threshold_c": "80"},
        {"temperature_c": float("nan")},
        {"threshold_c": float("-inf")},
        {"temperature_c": 10 ** 400},
        {"is_abnormal": 1},
        {"is_abnormal": "true"},
    ],
)
def test_invalid_result(changes):
    _assert_rejected(_message("result", **changes), "result")


@pytest.mark.parametrize("text", ["[]", "null", "true", "42"])
def test_json_must_contain_an_object(text):
    with pytest.raises(ValueError):
        decode_message(text, "status")


@pytest.mark.parametrize("text", ["", "not-json", '{"run_id":'])
def test_malformed_json(text):
    with pytest.raises(ValueError):
        decode_message(text, "result")


@pytest.mark.parametrize("value", [None, 123, {}])
def test_decoder_requires_text(value):
    with pytest.raises(ValueError):
        decode_message(value, "result")


def test_unknown_message_kind():
    _assert_rejected(_message("status"), "unknown")
