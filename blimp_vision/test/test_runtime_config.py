from pathlib import Path

from blimp_vision.blob_detector import BlobDetectorClass
from blimp_vision.runtime_config import apply_runtime_config, load_runtime_config_file
from blimp_vision import contour_goal_detection as goal_detection


def test_load_runtime_config_from_sample_yaml():
    sample = Path(__file__).resolve().parents[1] / "test_upload.yaml"
    config = load_runtime_config_file(sample)

    assert config.green_hsv.h_min == 41
    assert config.green_hsv.h_max == 80
    assert config.purple_hsv.h_min == 101
    assert config.goal_orange_hsv.v_min == 200
    assert config.goal_yellow_hsv.h_max == 40
    assert config.min_area == 220
    assert config.min_percent_filled == 50.0
    assert config.ignore_top_ratio == 0.0
    assert config.include_green is True
    assert config.include_purple is True
    assert config.use_kalman is True
    assert config.use_optical_flow is True
    assert config.use_lock is False
    assert config.goal_score_threshold == 0.18


def test_apply_runtime_config_updates_detector_modules():
    sample = Path(__file__).resolve().parents[1] / "test_upload.yaml"
    config = load_runtime_config_file(sample)
    detector = BlobDetectorClass()

    apply_runtime_config(config, detector, goal_detection)

    assert detector.green_lh == 41
    assert detector.green_uh == 80
    assert detector.purple_lh == 101
    assert detector.purple_uv == 190
    assert detector.min_area == 220
    assert detector.min_percent_filled == 50.0
    assert detector.ignore_top_ratio == 0.0
    assert detector.include_green is True
    assert detector.include_purple is True
    assert detector.use_kalman is True
    assert detector.use_optical_flow is True
    assert detector.use_lock is False
    assert goal_detection.orange_hsv["h_min"] == 2
    assert goal_detection.yellow_hsv["h_max"] == 40
    assert goal_detection.score_threshold == 0.18
