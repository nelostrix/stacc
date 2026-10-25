"""Scout rover: the body is described in RSS, the behaviour in Python.

Run it against a running STACC daemon:

    python examples/embedded/scout_rover.py

`rss(...)` compiles the block locally (so a mistake in the description fails
here, with the line in this file), and `load_file(__file__)` sends this file to
the daemon, which builds the same robot in MuJoCo.
"""

import math
import time

from stacc.rss import rss
from stacc.server.runtime_client import RuntimeClient

ROBOT = rss("""<<<RSS>>>
robot scout {
    chassis {
        shape: differential-drive;
        dimensions: [400mm, 300mm, 120mm];
        mass: 4kg;
        material: aluminum;
    }

    wheel front_left  { radius: 60mm; width: 30mm; mass: 0.3kg; material: rubber; effort: 1.2Nm; speed: 300rpm; }
    wheel front_right { radius: 60mm; width: 30mm; mass: 0.3kg; material: rubber; effort: 1.2Nm; speed: 300rpm; }
    wheel rear_left   { radius: 60mm; width: 30mm; mass: 0.3kg; material: rubber; effort: 1.2Nm; speed: 300rpm; }
    wheel rear_right  { radius: 60mm; width: 30mm; mass: 0.3kg; material: rubber; effort: 1.2Nm; speed: 300rpm; }

    sensors {
        imu body { model: bmi088; }
    }
}
<<</RSS>>>""")

# Each wheel motor stalls at 1.2 N·m and spins at most 300 rpm unloaded, so a
# steady 0.8 N·m settles at ~2/3 of that: about 21 rad/s, 1.3 m/s on 60 mm wheels.
WHEEL_TORQUE_NM = 0.8
DRIVE_SECONDS = 3.0


def main() -> None:
    wheels = [j.name for j in ROBOT.joints]
    print(f"{ROBOT.name}: {ROBOT.dof} joints ({', '.join(wheels)}), mobile={not ROBOT.fixed_base}")

    rt = RuntimeClient()
    rt.load_file(__file__)
    rt.start()
    time.sleep(1.0)  # settle onto the wheels

    start = rt.telemetry()["pose"]
    # Positive torque about each wheel's +Y axle rolls the rover towards +X.
    rt.command_torques([WHEEL_TORQUE_NM] * len(wheels))
    time.sleep(DRIVE_SECONDS)
    end = rt.telemetry()["pose"]
    rt.command_torques([0.0] * len(wheels))

    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    print(f"drove {distance:.2f} m in {DRIVE_SECONDS:.0f} s (x {start[0]:+.2f} -> {end[0]:+.2f} m, yaw {math.degrees(end[5]):+.1f} deg)")


if __name__ == "__main__":
    main()
