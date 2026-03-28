from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict

import yaml


@dataclass
class HSVRange:
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int


@dataclass
class RuntimeConfig:
    green_hsv: HSVRange = field(default_factory=lambda: HSVRange(39, 76, 78, 153, 58, 219))
    purple_hsv: HSVRange = field(default_factory=lambda: HSVRange(106, 133, 30, 189, 62, 180))
    goal_orange_hsv: HSVRange = field(default_factory=lambda: HSVRange(0, 25, 124, 255, 240, 255))
    goal_yellow_hsv: HSVRange = field(default_factory=lambda: HSVRange(27, 37, 107, 255, 197, 255))
    min_area: int = 250
    min_percent_filled: float = 60.0
    ignore_top_ratio: float = 0.0
    include_green: bool = True
    include_purple: bool = True
    include_goal_orange: bool = True
    include_goal_yellow: bool = True
    use_kalman: bool = True
    use_optical_flow: bool = True
    use_lock: bool = True
    goal_score_threshold: float = 0.2


def _node_block(doc: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(doc, dict):
        return {}
    for value in doc.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            params = value.get("ros__parameters")
            return params if isinstance(params, dict) else {}
    return {}


def _flatten_dict(data: Any, prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    if isinstance(data, dict):
        for key, value in data.items():
            full = f"{prefix}.{key}" if prefix else str(key)
            out.update(_flatten_dict(value, full))
    else:
        out[prefix] = data
    return out


def _first(flat: Dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in flat:
            return flat[key]
    return None


def _hsv_range(flat: Dict[str, Any], prefix: str, legacy_prefix: str, fallback: HSVRange) -> HSVRange:
    return HSVRange(
        h_min=int(_first(flat, f"{prefix}.h_min", f"{legacy_prefix}_h_min", f"{legacy_prefix}_lh") or fallback.h_min),
        h_max=int(_first(flat, f"{prefix}.h_max", f"{legacy_prefix}_h_max", f"{legacy_prefix}_uh") or fallback.h_max),
        s_min=int(_first(flat, f"{prefix}.s_min", f"{legacy_prefix}_s_min", f"{legacy_prefix}_ls") or fallback.s_min),
        s_max=int(_first(flat, f"{prefix}.s_max", f"{legacy_prefix}_s_max", f"{legacy_prefix}_us") or fallback.s_max),
        v_min=int(_first(flat, f"{prefix}.v_min", f"{legacy_prefix}_v_min", f"{legacy_prefix}_lv") or fallback.v_min),
        v_max=int(_first(flat, f"{prefix}.v_max", f"{legacy_prefix}_v_max", f"{legacy_prefix}_uv") or fallback.v_max),
    )


def load_runtime_config_file(path: str | Path) -> RuntimeConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as file:
        doc = yaml.safe_load(file) or {}

    params = _node_block(doc)
    flat = _flatten_dict(params)
    defaults = RuntimeConfig()
    include_green = _first(flat, "detectors.green_balloon.enabled", "include_green")
    include_purple = _first(flat, "detectors.purple_balloon.enabled", "include_purple")
    include_goal_orange = _first(flat, "detectors.goal_orange.enabled", "include_goal_orange")
    include_goal_yellow = _first(flat, "detectors.goal_yellow.enabled", "include_goal_yellow")
    use_kalman = _first(flat, "tracking.use_kalman", "use_kalman")
    use_optical_flow = _first(flat, "tracking.use_optical_flow", "use_optical_flow")
    use_lock = _first(flat, "tracking.use_lock", "use_lock")

    return RuntimeConfig(
        green_hsv=_hsv_range(flat, "detectors.green_balloon.hsv", "green", defaults.green_hsv),
        purple_hsv=_hsv_range(flat, "detectors.purple_balloon.hsv", "purple", defaults.purple_hsv),
        goal_orange_hsv=_hsv_range(flat, "detectors.goal_orange.hsv", "goal_orange", defaults.goal_orange_hsv),
        goal_yellow_hsv=_hsv_range(flat, "detectors.goal_yellow.hsv", "goal_yellow", defaults.goal_yellow_hsv),
        min_area=int(_first(
            flat,
            "detectors.green_balloon.filters.min_area",
            "detectors.purple_balloon.filters.min_area",
            "min_area",
        ) or defaults.min_area),
        min_percent_filled=float(_first(
            flat,
            "detectors.green_balloon.filters.min_fill",
            "detectors.purple_balloon.filters.min_fill",
            "min_percent_filled",
        ) or defaults.min_percent_filled),
        ignore_top_ratio=float(_first(
            flat,
            "detectors.green_balloon.filters.ignore_top_ratio",
            "detectors.purple_balloon.filters.ignore_top_ratio",
            "ignore_top_ratio",
        ) or defaults.ignore_top_ratio),
        include_green=bool(include_green) if include_green is not None else defaults.include_green,
        include_purple=bool(include_purple) if include_purple is not None else defaults.include_purple,
        include_goal_orange=bool(include_goal_orange) if include_goal_orange is not None else defaults.include_goal_orange,
        include_goal_yellow=bool(include_goal_yellow) if include_goal_yellow is not None else defaults.include_goal_yellow,
        use_kalman=bool(use_kalman) if use_kalman is not None else defaults.use_kalman,
        use_optical_flow=bool(use_optical_flow) if use_optical_flow is not None else defaults.use_optical_flow,
        use_lock=bool(use_lock) if use_lock is not None else defaults.use_lock,
        goal_score_threshold=float(_first(
            flat,
            "detectors.goal_orange.score_threshold",
            "detectors.goal_yellow.score_threshold",
            "goal_score_threshold",
        ) or defaults.goal_score_threshold),
    )


def apply_runtime_config(config: RuntimeConfig, blob_detector: Any, goal_module: Any) -> None:
    blob_detector.green_lh = config.green_hsv.h_min
    blob_detector.green_uh = config.green_hsv.h_max
    blob_detector.green_ls = config.green_hsv.s_min
    blob_detector.green_us = config.green_hsv.s_max
    blob_detector.green_lv = config.green_hsv.v_min
    blob_detector.green_uv = config.green_hsv.v_max

    blob_detector.purple_lh = config.purple_hsv.h_min
    blob_detector.purple_uh = config.purple_hsv.h_max
    blob_detector.purple_ls = config.purple_hsv.s_min
    blob_detector.purple_us = config.purple_hsv.s_max
    blob_detector.purple_lv = config.purple_hsv.v_min
    blob_detector.purple_uv = config.purple_hsv.v_max

    blob_detector.min_area = config.min_area
    blob_detector.min_percent_filled = config.min_percent_filled
    blob_detector.ignore_top_ratio = config.ignore_top_ratio
    blob_detector.include_green = config.include_green
    blob_detector.include_purple = config.include_purple
    blob_detector.use_kalman = config.use_kalman
    blob_detector.use_optical_flow = config.use_optical_flow
    blob_detector.use_lock = config.use_lock

    goal_module.orange_hsv["h_min"] = config.goal_orange_hsv.h_min
    goal_module.orange_hsv["h_max"] = config.goal_orange_hsv.h_max
    goal_module.orange_hsv["s_min"] = config.goal_orange_hsv.s_min
    goal_module.orange_hsv["s_max"] = config.goal_orange_hsv.s_max
    goal_module.orange_hsv["v_min"] = config.goal_orange_hsv.v_min
    goal_module.orange_hsv["v_max"] = config.goal_orange_hsv.v_max

    goal_module.yellow_hsv["h_min"] = config.goal_yellow_hsv.h_min
    goal_module.yellow_hsv["h_max"] = config.goal_yellow_hsv.h_max
    goal_module.yellow_hsv["s_min"] = config.goal_yellow_hsv.s_min
    goal_module.yellow_hsv["s_max"] = config.goal_yellow_hsv.s_max
    goal_module.yellow_hsv["v_min"] = config.goal_yellow_hsv.v_min
    goal_module.yellow_hsv["v_max"] = config.goal_yellow_hsv.v_max

    goal_module.score_threshold = config.goal_score_threshold
