"""Regression tests for bugs found during the Oct 2026 architecture review.

Run with:  python -m pytest tests/test_regressions.py
"""

import numpy as np
import pytest

from stacc.converters.urdf_mjcf_usd import _pose6


def test_pose6_pads_translation_only_origin():
    # Robots built in code often give just [x, y, z]; rotation defaults to zero.
    assert _pose6([1, 2, 3]) == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]


def test_pose6_handles_missing_and_full_origins():
    assert _pose6(None) == [0.0] * 6
    assert _pose6([]) == [0.0] * 6
    assert _pose6([1, 2, 3, 0.1, 0.2, 0.3]) == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]


def test_optical_flow_mean_motion_is_per_axis():
    cv2 = pytest.importorskip("cv2")
    from stacc.ai.vision import OpticalFlow

    rng = np.random.default_rng(0)
    base = (rng.random((120, 120, 3)) * 255).astype(np.uint8)
    base = cv2.GaussianBlur(base, (0, 0), 2)  # give the corner detector something trackable

    flow = OpticalFlow(method="sparse")
    flow.update(base)
    flow.update(np.roll(base, 4, axis=1))  # shift 4 px to the right

    dx, dy = flow.get_mean_motion()  # used to raise IndexError on (N, 1, 2) point arrays
    assert isinstance(dx, float) and isinstance(dy, float)
    assert dx > 1.0, "a rightward shift must show up in dx"
    assert abs(dy) < abs(dx), "a purely horizontal shift must not be reported as vertical motion"


def test_runtime_loop_stats_are_measured_not_configured():
    import time

    from stacc.server import StaccRuntime, create_robot

    rt = StaccRuntime(physics="mujoco", hz=500.0, timestep=0.002, headless=True)
    rt.set_robot(create_robot("arm", name="timing-test"))
    assert rt.loop_stats()["samples"] == 0  # nothing measured before the loop has run
    rt.start()
    try:
        time.sleep(1.0)
        stats = rt.loop_stats()
    finally:
        rt.stop()

    assert stats["samples"] > 50
    assert 0 < stats["measured_hz"] <= 600, "measured rate must reflect the real loop, not the configured one"
    assert stats["jitter_ms"] >= 0.0
    assert stats["period_max_ms"] >= stats["period_p99_ms"] >= 0.0
