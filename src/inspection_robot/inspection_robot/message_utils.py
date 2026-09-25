"""Encode and validate the JSON payloads carried by ROS String messages."""

import json
import math


STATES = ("WAITING", "MOVING", "INSPECTING", "COMPLETED", "ERROR")


def encode_message(data, kind):
    """Validate a status or result dictionary and return JSON text."""
    _validate_message(data, kind)
    return json.dumps(data, ensure_ascii=False, allow_nan=False)


def decode_message(text, kind):
    """Return a validated dictionary; raise ValueError for invalid input."""
    if not isinstance(text, str):
        raise ValueError("Message text must be a string.")
    data = json.loads(text)
    _validate_message(data, kind)
    return data


def _validate_message(data, kind):
    common = {"run_id", "point_id", "timestamp"}
    if kind == "status":
        required = common | {"state", "x", "y", "theta", "detail"}
    elif kind == "result":
        required = common | {"temperature_c", "threshold_c", "is_abnormal"}
    else:
        raise ValueError("Message kind must be 'status' or 'result'.")

    if not isinstance(data, dict) or set(data) != required:
        raise ValueError(f"{kind} message has missing or unexpected fields.")
    for key in ("run_id", "point_id"):
        if not isinstance(data[key], str) or not data[key].strip():
            raise ValueError(f"{key} must be a non-empty string.")
    _require_number(data["timestamp"], "timestamp")
    if data["timestamp"] < 0:
        raise ValueError("timestamp must be non-negative Unix time.")

    if kind == "status":
        if data["state"] not in STATES:
            raise ValueError("Unknown patrol state.")
        if not isinstance(data["detail"], str):
            raise ValueError("detail must be a string.")
        pose_keys = ("x", "y", "theta")
        if all(data[key] is None for key in pose_keys):
            if data["state"] not in ("WAITING", "ERROR"):
                raise ValueError("This state requires a known pose.")
        else:
            for key in pose_keys:
                _require_number(data[key], key)
    else:
        _require_number(data["temperature_c"], "temperature_c")
        _require_number(data["threshold_c"], "threshold_c")
        if type(data["is_abnormal"]) is not bool:
            raise ValueError("is_abnormal must be a boolean.")
        expected = data["temperature_c"] > data["threshold_c"]
        if data["is_abnormal"] != expected:
            raise ValueError("is_abnormal disagrees with the temperature.")


def _require_number(value, field):
    if type(value) not in (int, float):
        raise ValueError(f"{field} must be a number, not a boolean or string.")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{field} must be finite.")
