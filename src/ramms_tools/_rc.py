"""
Small helpers for the values Unreal's Remote Control API hands back.

Shared by ramms_tools.control, .robot and .sensing. Unreal returns:

- a function's out parameters beside its ``ReturnValue`` (see
  ``UnrealRemote.call_full``), and no ``ReturnValue`` for a void function;
- enums as their names (``"Idle"``, ``"Info"``; sometimes qualified,
  ``"ERammsControlTextSeverity::Info"``);
- ``FVector2D`` as ``{"X", "Y"}``, ``FVector`` as ``{"X", "Y", "Z"}``;
- ``FName`` ``None`` as the string ``"None"``;
- JSON-returning bridge functions as a JSON string.
"""

from __future__ import annotations

import json
from typing import Any


def return_value(result: Any, default: Any = None) -> Any:
    """The ReturnValue of a full response (or the value itself when it is not wrapped)."""
    if isinstance(result, dict) and "ReturnValue" in result:
        return result["ReturnValue"]
    return default if result is None else result


def as_json(result: Any) -> Any:
    """A bridge function's JSON result: unwraps ReturnValue and parses the string ("" -> None)."""
    result = return_value(result)
    if isinstance(result, (bytes, bytearray)):
        result = result.decode("utf-8")
    if isinstance(result, str):
        return json.loads(result) if result.strip() else None
    return result


def name(value: Any) -> str:
    """An FName: Unreal's "None" (and None) becomes ""."""
    s = "" if value is None else str(value)
    return "" if s == "None" else s


def enum_name(value: Any) -> str:
    """An enum as Unreal hands it back: "ERammsCurbPhase::Done" -> "Done"."""
    s = name(value)
    return s.rsplit("::", 1)[-1]


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")
    return bool(value)


def vec2(value: Any) -> tuple[float, float]:
    """An FVector2D ({"X", "Y"} or {"x", "y"} or a pair) as a tuple."""
    if isinstance(value, dict):
        return (as_float(value.get("X", value.get("x", 0.0))), as_float(value.get("Y", value.get("y", 0.0))))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return (as_float(value[0]), as_float(value[1]))
    return (0.0, 0.0)


def vec3(value: Any) -> tuple[float, float, float]:
    """An FVector ({"X", "Y", "Z"} or lower case, or a triple) as a tuple."""
    if isinstance(value, dict):
        g = lambda k: as_float(value.get(k.upper(), value.get(k, 0.0)))
        return (g("x"), g("y"), g("z"))
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (as_float(value[0]), as_float(value[1]), as_float(value[2]))
    return (0.0, 0.0, 0.0)


def rotator(value: Any) -> tuple[float, float, float]:
    """An FRotator as (pitch, yaw, roll) from {"Pitch", "Yaw", "Roll"} (either case) or a triple."""
    if isinstance(value, dict):
        g = lambda k: as_float(value.get(k.capitalize(), value.get(k, 0.0)))
        return (g("pitch"), g("yaw"), g("roll"))
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (as_float(value[0]), as_float(value[1]), as_float(value[2]))
    return (0.0, 0.0, 0.0)


def short_name(path: str) -> str:
    """The last part of an object path: ".../PersistentLevel.BP_Robot_C_0" -> "BP_Robot_C_0"."""
    s = str(path or "")
    return s.rsplit(".", 1)[-1].rsplit(":", 1)[-1]
