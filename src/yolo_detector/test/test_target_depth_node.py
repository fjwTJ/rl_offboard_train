import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from yolo_detector.target_depth_node import TargetDepthNode


def make_estimator(depth_value_type='z_depth'):
    estimator = TargetDepthNode.__new__(TargetDepthNode)
    estimator.fx = 100.0
    estimator.fy = 100.0
    estimator.cx = 50.0
    estimator.cy = 50.0
    estimator.depth_value_type = depth_value_type
    estimator.foreground_percentile = 25.0
    estimator.depth_tolerance_mm = 250.0
    estimator.min_depth_m = 0.1
    estimator.max_depth_m = 30.0
    estimator.min_foreground_pixels = 4
    estimator.max_depth_jump_m = 1.5
    estimator.depth_filter_alpha = 0.45
    estimator._filtered_estimate = None
    return estimator


def make_detection(cx, cy, w, h, score=0.9):
    hypothesis = SimpleNamespace(score=score)
    result = SimpleNamespace(hypothesis=hypothesis)
    center = SimpleNamespace(position=SimpleNamespace(x=float(cx), y=float(cy)))
    bbox = SimpleNamespace(center=center, size_x=float(w), size_y=float(h))
    return SimpleNamespace(bbox=bbox, results=[result])


def put_target(depth, cx, cy, z_m, radius=2):
    y0 = max(0, cy - radius)
    y1 = min(depth.shape[0], cy + radius + 1)
    x0 = max(0, cx - radius)
    x1 = min(depth.shape[1], cx + radius + 1)
    depth[y0:y1, x0:x1] = z_m


def test_center_target_depth_from_foreground_cluster():
    estimator = make_estimator()
    depth = np.full((100, 100), 8.0, dtype=np.float32)
    put_target(depth, 50, 50, 3.0)

    estimate = estimator.estimate_target_pixel_and_depth(
        depth,
        make_detection(50, 50, 30, 30),
    )

    assert estimate is not None
    px, py, z_m = estimate
    assert abs(px - 50) <= 1
    assert abs(py - 50) <= 1
    assert z_m == 3.0


def test_ray_range_edge_target_is_converted_to_z_depth():
    estimator = make_estimator(depth_value_type='ray_range')
    true_z = 4.0
    edge_x = 85
    edge_y = 50
    x_norm = (edge_x - estimator.cx) / estimator.fx
    y_norm = (edge_y - estimator.cy) / estimator.fy
    ray_range = true_z * math.sqrt(1.0 + x_norm * x_norm + y_norm * y_norm)
    depth = np.full((100, 100), 10.0, dtype=np.float32)
    put_target(depth, edge_x, edge_y, ray_range)

    depth_z_m = estimator.depth_image_to_meters(depth)
    estimate = estimator.estimate_target_pixel_and_depth(
        depth_z_m,
        make_detection(edge_x, edge_y, 24, 24),
    )

    assert estimate is not None
    assert abs(estimate[2] - true_z) < 0.03


def test_foreground_cluster_wins_over_far_background():
    estimator = make_estimator()
    depth = np.full((100, 100), 12.0, dtype=np.float32)
    put_target(depth, 40, 42, 2.5, radius=3)

    estimate = estimator.estimate_target_pixel_and_depth(
        depth,
        make_detection(50, 50, 50, 50),
    )

    assert estimate is not None
    assert abs(estimate[2] - 2.5) < 0.01


def test_too_few_foreground_pixels_returns_none():
    estimator = make_estimator()
    estimator.min_foreground_pixels = 6
    depth = np.full((100, 100), 9.0, dtype=np.float32)
    depth[49:51, 49:51] = 3.0

    estimate = estimator.estimate_target_pixel_and_depth(
        depth,
        make_detection(50, 50, 20, 20),
    )

    assert estimate is None


def test_temporal_filter_rejects_sudden_far_jump():
    estimator = make_estimator()

    first = estimator.apply_temporal_filter((50, 50, 3.0))
    second = estimator.apply_temporal_filter((50, 50, 6.0))

    assert first == (50, 50, 3.0)
    assert second is None
