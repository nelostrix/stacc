"""Test all AI modules — SLAM, vision, NLP."""

import math
import sys

results = []


def _run_legacy_check(name, func):
    if __name__ != "__main__":
        return
    try:
        func()
        results.append((name, "PASS"))
        print(f"  \033[92m✓\033[0m {name}")
    except Exception as e:
        results.append((name, "FAIL"))
        print(f"  \033[91m✗\033[0m {name}: {e}")


print("\n" + "=" * 60)
print("  stacc AI module tests")
print("=" * 60 + "\n")

# ═══════ EKF-SLAM ═══════


def test_ekf_slam():
    from stacc.ai.slam import EKFSLAM

    slam = EKFSLAM()
    slam.predict(1.0, 0.1, 0.1)
    assert abs(slam.x) > 0 or abs(slam.y) > 0
    slam.update([(2.0, 0.5, -1)])
    assert slam.num_landmarks == 1
    lms = slam.landmarks
    assert len(lms) == 1


_run_legacy_check("EKF-SLAM — predict + update", test_ekf_slam)


def test_ekf_slam_multi():
    from stacc.ai.slam import EKFSLAM

    slam = EKFSLAM()
    for i in range(10):
        slam.predict(0.5, 0.05, 0.1)
        slam.update([(1.5 + i * 0.1, 0.3, -1)])
    assert slam.num_landmarks > 0
    assert abs(slam.x) > 0


_run_legacy_check("EKF-SLAM — multi-step", test_ekf_slam_multi)

# ═══════ PARTICLE FILTER SLAM ═══════


def test_particle_slam():
    from stacc.ai.slam import ParticleFilterSLAM

    pf = ParticleFilterSLAM(num_particles=50)
    pf.predict(1.0, 0.0, 0.1)
    pose = pf.pose
    assert len(pose) == 3
    pf.update([(2.0, 0.5, 0)])
    mean = pf.mean_pose
    assert len(mean) == 3


_run_legacy_check("Particle Filter SLAM", test_particle_slam)

# ═══════ ICP ═══════


def test_icp():
    import numpy as np

    from stacc.ai.slam import ICPMatcher

    source = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    target = source + np.array([0.5, 0.3])
    icp = ICPMatcher(max_iterations=50)
    R, t, fitness = icp.match(source, target)
    assert abs(t[0] - 0.5) < 0.2
    assert abs(t[1] - 0.3) < 0.2


_run_legacy_check("ICP scan matching", test_icp)

# ═══════ OCCUPANCY GRID MAPPER ═══════


def test_occ_mapper():
    from stacc.ai.slam import OccupancyGridMapper

    mapper = OccupancyGridMapper(width=100, height=100, resolution=0.1)
    ranges = [5.0] * 36
    ranges[0] = 2.0
    mapper.update([5.0, 5.0, 0.0], ranges, angle_min=-math.pi, angle_max=math.pi)
    binary = mapper.get_binary_map()
    assert binary.shape == (100, 100)


_run_legacy_check("Occupancy grid mapper", test_occ_mapper)

# ═══════ SLAM NAVIGATOR ═══════


def test_slam_navigator():
    from stacc.ai.slam import SLAMNavigator
    from stacc.core.link import Link
    from stacc.core.robot_base import Robot

    robot = Robot("nav_test")
    robot.add_link(Link("base", mass=1.0))
    slam = SLAMNavigator(method="ekf")
    slam.initialize(robot)
    pose = slam.get_pose()
    assert len(pose) == 3


_run_legacy_check("SLAM Navigator", test_slam_navigator)

# ═══════ NLP COMMAND PARSER ═══════


def test_nlp_move():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("move forward 3 meters")
    assert cmd.action == "move"
    assert cmd.params["direction"] == "forward"
    assert cmd.params["distance"] == 3.0


_run_legacy_check("NLP — move command", test_nlp_move)


def test_nlp_turn():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("turn left 45 degrees")
    assert cmd.action == "turn"
    assert cmd.params["direction"] == "left"
    assert cmd.params["angle"] == 45.0


_run_legacy_check("NLP — turn command", test_nlp_turn)


def test_nlp_stop():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("stop")
    assert cmd.action == "stop"


_run_legacy_check("NLP — stop command", test_nlp_stop)


def test_nlp_pick():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("pick up the red cube")
    assert cmd.action == "pick"
    assert "red cube" in cmd.params["object"]


_run_legacy_check("NLP — pick command", test_nlp_pick)


def test_nlp_goto():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("go to position 3.5, 2.1")
    assert cmd.action == "goto"
    assert abs(cmd.params["x"] - 3.5) < 0.01
    assert abs(cmd.params["y"] - 2.1) < 0.01


_run_legacy_check("NLP — goto position", test_nlp_goto)


def test_nlp_unknown():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    cmd = parser.parse("xyzzy blah blah")
    assert cmd.action == "unknown"
    assert cmd.confidence == 0.0


_run_legacy_check("NLP — unknown command", test_nlp_unknown)


def test_nlp_custom_pattern():
    from stacc.ai.nlp import CommandParser

    parser = CommandParser()
    parser.add_pattern(r"dance\s+(\w+)", "dance", lambda m: {"style": m.group(1)})
    cmd = parser.parse("dance salsa")
    assert cmd.action == "dance"
    assert cmd.params["style"] == "salsa"


_run_legacy_check("NLP — custom pattern", test_nlp_custom_pattern)


def test_nlp_action_mapper():
    from stacc.ai.nlp import ActionMapper, CommandParser, ParsedCommand
    from stacc.core.link import Link
    from stacc.core.robot_base import Robot

    robot = Robot("nlp_bot")
    robot.add_link(Link("base", mass=1.0))
    mapper = ActionMapper(robot)
    called = [False]

    def custom_handler(r, p):
        called[0] = True

    mapper.register("test_action", custom_handler)
    mapper.execute(ParsedCommand(action="test_action"))
    assert called[0]


_run_legacy_check("NLP — action mapper", test_nlp_action_mapper)

# ═══════ VISION (without OpenCV) ═══════


def test_detection_class():
    from stacc.ai.vision import Detection

    d = Detection(
        label="cup", confidence=0.95, bbox=[10, 20, 50, 60], center=[35, 50], area=3000
    )
    assert d.label == "cup"
    dd = d.to_dict()
    assert dd["confidence"] == 0.95


_run_legacy_check("Vision — Detection class", test_detection_class)


def test_object_detector_init():
    from stacc.ai.vision import ObjectDetector

    det = ObjectDetector(backend="opencv")
    assert det.backend == "opencv"


_run_legacy_check("Vision — ObjectDetector init", test_object_detector_init)


def test_feature_extractor_init():
    from stacc.ai.vision import FeatureExtractor

    fe = FeatureExtractor(method="orb", max_features=500)
    assert fe.method == "orb"


_run_legacy_check("Vision — FeatureExtractor init", test_feature_extractor_init)


def test_color_detector():
    from stacc.ai.vision import ColorDetector

    cd = ColorDetector()
    cd.add_color("red", [0, 100, 100], [10, 255, 255])
    cd.add_color("blue", [100, 100, 100], [130, 255, 255])
    assert len(cd.colors) == 2


_run_legacy_check("Vision — ColorDetector setup", test_color_detector)


def test_visual_odometry_init():
    from stacc.ai.vision import VisualOdometry

    vo = VisualOdometry(method="orb")
    assert vo.position == [0, 0, 0]


_run_legacy_check("Vision — VisualOdometry init", test_visual_odometry_init)


def test_stereo_depth_init():
    from stacc.ai.vision import StereoDepthEstimator

    sde = StereoDepthEstimator(baseline=0.12, focal_length=500)
    assert sde.baseline == 0.12


_run_legacy_check("Vision — StereoDepthEstimator init", test_stereo_depth_init)

# ═══════ VISION WITH OPENCV ═══════


def test_vision_opencv():
    try:
        import cv2
        import numpy as np
    except ImportError:
        return  # skip

    from stacc.ai.vision import FeatureExtractor, ObjectDetector

    img = np.zeros((200, 300, 3), dtype=np.uint8)
    cv2.rectangle(img, (50, 50), (150, 150), (255, 255, 255), -1)

    det = ObjectDetector(backend="opencv")
    detections = det.detect(img)
    assert len(detections) > 0

    fe = FeatureExtractor(method="orb")
    kp, desc = fe.extract(img)


_run_legacy_check("Vision — OpenCV detection + features", test_vision_opencv)


def test_optical_flow():
    try:
        import cv2
        import numpy as np
    except ImportError:
        return

    from stacc.ai.vision import OpticalFlow

    flow = OpticalFlow(method="sparse")
    frame1 = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
    flow.update(frame1)
    frame2 = np.roll(frame1, 5, axis=1)
    flow.update(frame2)
    dx, dy = flow.get_mean_motion()


_run_legacy_check("Vision — optical flow", test_optical_flow)

# ═══════ SUMMARY ═══════

print("\n" + "=" * 60)
passed = sum(1 for _, s in results if s == "PASS")
failed = sum(1 for _, s in results if s == "FAIL")
total = len(results)
if failed == 0:
    print(f"  \033[92mALL {total} TESTS PASSED\033[0m")
else:
    print(
        f"  \033[92m{passed} passed\033[0m, \033[91m{failed} failed\033[0m out of {total}"
    )
print("=" * 60 + "\n")
if __name__ == "__main__":
    sys.exit(1 if failed > 0 else 0)
