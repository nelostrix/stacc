"""Master test — tests world, recorder, behavior tree, HAL, path planning, trajectory, gym env."""

import json
import math
import os
import sys
import tempfile

results = []


def test(name, func):
    try:
        func()
        results.append((name, "PASS", ""))
        print(f"  \033[92m✓\033[0m {name}")
    except Exception as e:
        results.append((name, "FAIL", str(e)))
        print(f"  \033[91m✗\033[0m {name}: {e}")


print("\n" + "=" * 60)
print("  stacc master test — all modules")
print("=" * 60 + "\n")

# ═══════ WORLD ═══════


def test_world_basic():
    from stacc.sim.world import Terrain, World

    w = World("test")
    w.add_box("box1", [1, 0, 0.5], [0.2, 0.2, 0.2], mass=1.0)
    w.add_sphere("sphere1", [2, 0, 0.3], radius=0.1)
    w.add_cylinder("cyl1", [3, 0, 0.5], radius=0.05, length=1.0)
    assert len(w.objects) == 3
    els = w.to_mjcf_elements()
    assert len(els) == 3


test("World — add objects", test_world_basic)


def test_world_wall():
    from stacc.sim.world import World

    w = World()
    w.add_wall("wall1", [0, 0], [5, 0], height=2.0)
    assert len(w.objects) == 1
    assert w.objects[0].shape == "box"


test("World — walls", test_world_wall)


def test_world_terrain():
    from stacc.sim.world import Terrain, World

    w = World()
    w.set_terrain(Terrain.stairs(num_steps=5, step_height=0.1))
    els = w.to_mjcf_elements()
    assert len(els) == 5


test("World — terrain stairs", test_world_terrain)


def test_world_prebuilt():
    from stacc.sim.world import (
        mars_world,
        obstacle_course,
        tabletop_world,
        warehouse_world,
    )

    for factory in [warehouse_world, obstacle_course, tabletop_world, mars_world]:
        w = factory()
        assert len(w.objects) > 0
        assert w.name != ""


test("World — pre-built environments", test_world_prebuilt)

# ═══════ RECORDER ═══════


def test_recorder_basic():
    from stacc.core.joint import Joint
    from stacc.core.link import Link
    from stacc.core.robot_base import Robot
    from stacc.sim.recorder import SimRecorder

    robot = Robot("rec_test")
    robot.add_link(Link("base", mass=1.0))
    robot.add_joint(Joint("j1", type="revolute"))
    robot.add_link(Link("arm", mass=0.5))

    recorder = SimRecorder(robot)
    for i in range(100):
        robot._state["joint_positions"][0] = math.sin(i * 0.1)
        robot.time = i * 0.01
        recorder.record()

    assert recorder.recording.num_frames == 100
    assert recorder.recording.duration > 0


test("Recorder — basic recording", test_recorder_basic)


def test_recorder_save_load():
    from stacc.core.joint import Joint
    from stacc.core.link import Link
    from stacc.core.robot_base import Robot
    from stacc.sim.recorder import SimRecorder, load_recording

    robot = Robot("save_test")
    robot.add_link(Link("base", mass=1.0))
    robot.add_joint(Joint("j1", type="revolute"))
    robot.add_link(Link("arm", mass=0.5))

    recorder = SimRecorder(robot)
    for i in range(50):
        robot._state["joint_positions"][0] = i * 0.1
        robot.time = i * 0.01
        recorder.record()

    tmpfile = tempfile.mktemp(suffix=".json")
    recorder.save(tmpfile)
    assert os.path.exists(tmpfile)

    loaded = load_recording(tmpfile)
    assert loaded.num_frames == 50
    os.unlink(tmpfile)


test("Recorder — save/load JSON", test_recorder_save_load)


def test_recorder_csv():
    from stacc.core.joint import Joint
    from stacc.core.link import Link
    from stacc.core.robot_base import Robot
    from stacc.sim.recorder import SimRecorder

    robot = Robot("csv_test")
    robot.add_link(Link("base", mass=1.0))
    robot.add_joint(Joint("j1", type="revolute"))
    robot.add_link(Link("arm", mass=0.5))

    recorder = SimRecorder(robot)
    for i in range(20):
        robot._state["joint_positions"][0] = i * 0.05
        robot.time = i * 0.01
        recorder.record()

    tmpfile = tempfile.mktemp(suffix=".csv")
    recorder.save_csv(tmpfile)
    assert os.path.exists(tmpfile)

    with open(tmpfile) as f:
        lines = f.readlines()
    assert len(lines) == 21
    os.unlink(tmpfile)


test("Recorder — save CSV", test_recorder_csv)

# ═══════ BEHAVIOR TREE ═══════


def test_bt_action():
    from stacc.ai.behavior_tree import Action, Status

    call_count = [0]

    def my_action(dt):
        call_count[0] += 1
        return Status.SUCCESS if call_count[0] >= 3 else Status.RUNNING

    node = Action("test", my_action)
    assert node.tick() == Status.RUNNING
    assert node.tick() == Status.RUNNING
    assert node.tick() == Status.SUCCESS


test("BehaviorTree — Action node", test_bt_action)


def test_bt_condition():
    from stacc.ai.behavior_tree import Condition, Status

    val = [False]
    node = Condition("check", lambda dt: val[0])
    assert node.tick() == Status.FAILURE
    val[0] = True
    assert node.tick() == Status.SUCCESS


test("BehaviorTree — Condition node", test_bt_condition)


def test_bt_sequence():
    from stacc.ai.behavior_tree import Action, Sequence, Status

    seq = Sequence(
        "seq",
        [
            Action("a1", lambda dt: Status.SUCCESS),
            Action("a2", lambda dt: Status.SUCCESS),
            Action("a3", lambda dt: Status.SUCCESS),
        ],
    )
    assert seq.tick() == Status.SUCCESS


test("BehaviorTree — Sequence (all succeed)", test_bt_sequence)


def test_bt_sequence_fail():
    from stacc.ai.behavior_tree import Action, Sequence, Status

    seq = Sequence(
        "seq",
        [
            Action("a1", lambda dt: Status.SUCCESS),
            Action("a2", lambda dt: Status.FAILURE),
            Action("a3", lambda dt: Status.SUCCESS),
        ],
    )
    assert seq.tick() == Status.FAILURE


test("BehaviorTree — Sequence (one fails)", test_bt_sequence_fail)


def test_bt_selector():
    from stacc.ai.behavior_tree import Action, Selector, Status

    sel = Selector(
        "sel",
        [
            Action("a1", lambda dt: Status.FAILURE),
            Action("a2", lambda dt: Status.SUCCESS),
            Action("a3", lambda dt: Status.FAILURE),
        ],
    )
    assert sel.tick() == Status.SUCCESS


test("BehaviorTree — Selector", test_bt_selector)


def test_bt_repeat():
    from stacc.ai.behavior_tree import Action, Repeat, Status

    count = [0]

    def inc(dt):
        count[0] += 1
        return Status.SUCCESS

    node = Repeat("rep", Action("a", inc), count=3)
    node.tick()
    node.tick()
    result = node.tick()
    assert result == Status.SUCCESS
    assert count[0] == 3


test("BehaviorTree — Repeat decorator", test_bt_repeat)


def test_bt_timeout():
    from stacc.ai.behavior_tree import Action, Status, Timeout

    node = Timeout("to", Action("slow", lambda dt: Status.RUNNING), time_limit=1.0)
    assert node.tick(0.3) == Status.RUNNING
    assert node.tick(0.3) == Status.RUNNING
    assert node.tick(0.5) == Status.FAILURE


test("BehaviorTree — Timeout decorator", test_bt_timeout)


def test_bt_wait():
    from stacc.ai.behavior_tree import Status, Wait

    node = Wait("wait", duration=0.5)
    assert node.tick(0.2) == Status.RUNNING
    assert node.tick(0.2) == Status.RUNNING
    assert node.tick(0.2) == Status.SUCCESS


test("BehaviorTree — Wait node", test_bt_wait)

# ═══════ PATH PLANNING ═══════


def test_astar():
    from stacc.ai.path_planning import AStarPlanner, OccupancyGrid

    grid = OccupancyGrid(50, 50, resolution=0.1, origin=[0, 0])
    grid.add_rectangle(1.0, 0.5, 0.5, 2.0)
    planner = AStarPlanner(grid)
    path = planner.plan([0.2, 0.2], [3.0, 2.0])
    assert path is not None
    assert len(path) >= 2


test("PathPlanning — A*", test_astar)


def test_astar_no_path():
    from stacc.ai.path_planning import AStarPlanner, OccupancyGrid

    grid = OccupancyGrid(20, 20, resolution=0.1)
    for x in range(20):
        grid.set_occupied(x, 10)
    planner = AStarPlanner(grid)
    path = planner.plan([0.5, 0.5], [0.5, 1.5])
    assert path is None


test("PathPlanning — A* no path", test_astar_no_path)


def test_dijkstra():
    from stacc.ai.path_planning import DijkstraPlanner, OccupancyGrid

    grid = OccupancyGrid(30, 30, resolution=0.1)
    planner = DijkstraPlanner(grid)
    path = planner.plan([0.5, 0.5], [2.0, 2.0])
    assert path is not None
    assert len(path) >= 2


test("PathPlanning — Dijkstra", test_dijkstra)


def test_rrt_star():
    from stacc.ai.path_planning import RRTStarPlanner

    planner = RRTStarPlanner(
        bounds=[[0, 5], [0, 5]],
        obstacle_check=lambda x, y: 1.5 < x < 2.5 and 0 < y < 3,
        step_size=0.3,
        max_iterations=2000,
        goal_bias=0.15,
    )
    path = planner.plan([0.5, 0.5], [4.0, 4.0])
    assert path is not None
    assert len(path) >= 2


test("PathPlanning — RRT*", test_rrt_star)


def test_potential_field():
    from stacc.ai.path_planning import OccupancyGrid, PotentialFieldPlanner

    grid = OccupancyGrid(40, 40, resolution=0.1)
    grid.add_circle(2.0, 1.5, 0.3)
    planner = PotentialFieldPlanner(grid, step_size=0.05, max_iterations=500)
    path = planner.plan([0.5, 0.5], [3.5, 2.5])
    assert path is not None
    assert len(path) > 5


test("PathPlanning — Potential Fields", test_potential_field)


def test_occupancy_grid():
    from stacc.ai.path_planning import OccupancyGrid

    grid = OccupancyGrid(100, 100, resolution=0.05, origin=[-2.5, -2.5])
    grid.add_rectangle(0, 0, 1, 1)
    grid.add_circle(2, 2, 0.5)
    gx, gy = grid.world_to_grid(0.5, 0.5)
    assert grid.is_occupied(gx, gy)
    inflated = grid.inflate(0.1)
    assert inflated.width == 100


test("PathPlanning — OccupancyGrid ops", test_occupancy_grid)

# ═══════ TRAJECTORY ═══════


def test_quintic():
    from stacc.kinematics.trajectory import QuinticTrajectory

    traj = QuinticTrajectory(q_start=0.0, q_end=1.5, duration=2.0)
    p0, v0, a0 = traj.evaluate(0.0)
    assert abs(p0 - 0.0) < 0.01
    p1, v1, a1 = traj.evaluate(2.0)
    assert abs(p1 - 1.5) < 0.01
    assert abs(v1) < 0.01


test("Trajectory — Quintic", test_quintic)


def test_minjerk():
    from stacc.kinematics.trajectory import MinJerkTrajectory

    traj = MinJerkTrajectory([0, 0], [1, -1], duration=3.0)
    p, v, a = traj.evaluate(1.5)
    assert len(p) == 2
    assert 0 < p[0] < 1
    assert -1 < p[1] < 0


test("Trajectory — MinJerk", test_minjerk)


def test_trapezoidal():
    from stacc.kinematics.trajectory import TrapezoidalProfile

    traj = TrapezoidalProfile(
        q_start=0.0, q_end=2.0, max_velocity=1.0, max_acceleration=5.0
    )
    p0, _, _ = traj.evaluate(0.0)
    pf, _, _ = traj.evaluate(traj.duration)
    assert abs(p0 - 0.0) < 0.01
    assert abs(pf - 2.0) < 0.05


test("Trajectory — Trapezoidal", test_trapezoidal)


def test_waypoint_path():
    from stacc.kinematics.trajectory import WaypointPath

    path = WaypointPath(
        waypoints=[[0, 0], [1, 0.5], [0.5, 1], [0, 0]],
        durations=[1.0, 1.0, 1.0],
        method="minjerk",
    )
    assert path.num_segments == 3
    assert abs(path.duration - 3.0) < 0.01
    p, v, a = path.evaluate(1.5)
    assert len(p) == 2


test("Trajectory — WaypointPath", test_waypoint_path)

# ═══════ HAL ═══════


def test_board_detect():
    from stacc.hal.board import Board

    board = Board.detect()
    assert board.platform in ["raspberry_pi", "jetson", "desktop", "unknown"]
    info = board.info()
    assert "platform" in info


test("HAL — Board detection", test_board_detect)


def test_gpio_simulation():
    from stacc.hal.gpio import GPIO

    gpio = GPIO(simulation=True)
    gpio.setup(18, GPIO.OUTPUT)
    gpio.write(18, GPIO.HIGH)
    assert gpio.read(18) == GPIO.HIGH
    gpio.write(18, GPIO.LOW)
    assert gpio.read(18) == GPIO.LOW


test("HAL — GPIO (simulation mode)", test_gpio_simulation)


def test_serial_list():
    from stacc.hal.serial_comm import SerialPort

    ports = SerialPort.list_ports()
    assert isinstance(ports, list)


test("HAL — Serial port listing", test_serial_list)


def test_motor_simulation():
    from stacc.hal.gpio import GPIO
    from stacc.hal.motor import MotorDriver

    gpio = GPIO(simulation=True)
    motor = MotorDriver(pin_a=17, pin_b=27, gpio=gpio)
    motor.set_speed(0.5)
    assert motor.speed == 0.5
    motor.set_speed(-0.3)
    assert abs(motor.speed - (-0.3)) < 0.01
    motor.stop()
    assert motor.speed == 0.0


test("HAL — MotorDriver (simulation)", test_motor_simulation)


def test_servo_simulation():
    from stacc.hal.gpio import GPIO
    from stacc.hal.motor import ServoDriver

    gpio = GPIO(simulation=True)
    servo = ServoDriver(pin=18, gpio=gpio)
    servo.set_angle(90)
    assert servo.angle == 90
    servo.set_angle(0)
    assert servo.angle == 0
    servo.set_angle(200)
    assert servo.angle == 180


test("HAL — ServoDriver (simulation)", test_servo_simulation)

# ═══════ GYM ENV ═══════


def test_gym_env():
    try:
        import numpy as np
    except ImportError:
        return

    from stacc.core.joint import Joint
    from stacc.core.link import Geometry, Link
    from stacc.core.robot_base import Robot
    from stacc.sim.gym_env import StaccEnv

    robot = Robot("gym_test")
    robot.add_link(
        Link("base", mass=2.0, visual=Geometry("box", size=[0.1, 0.1, 0.05]))
    )
    robot.add_joint(Joint("j1", type="revolute"))
    robot.add_link(
        Link("arm", mass=1.0, visual=Geometry("box", size=[0.04, 0.04, 0.2]))
    )

    env = StaccEnv(robot, max_steps=100, render=False)
    obs, info = env.reset()
    assert len(obs) > 0

    action = np.zeros(env.action_space.shape)
    obs, reward, terminated, truncated, info = env.step(action)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    env.close()


test("GymEnv — basic step cycle", test_gym_env)

# ═══════ SUMMARY ═══════

print("\n" + "=" * 60)
passed = sum(1 for _, s, _ in results if s == "PASS")
failed = sum(1 for _, s, _ in results if s == "FAIL")
total = len(results)

if failed == 0:
    print(f"  \033[92mALL {total} TESTS PASSED\033[0m")
else:
    print(
        f"  \033[92m{passed} passed\033[0m, \033[91m{failed} failed\033[0m out of {total}"
    )
    print()
    for name, status, err in results:
        if status == "FAIL":
            print(f"  \033[91m✗ {name}\033[0m")
            print(f"    {err}")

print("=" * 60 + "\n")
sys.exit(1 if failed > 0 else 0)
