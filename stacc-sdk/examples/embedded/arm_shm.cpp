// Arm over shared memory: a 1 kHz lockstep control loop in C++.
//
//   g++ -std=c++17 -O2 -I bindings/cpp examples/embedded/arm_shm.cpp -o arm_shm && ./arm_shm
//
// HTTP sets things up (load this file's robot, enable the channel); every
// control step then goes through /dev/shm — no sockets, no JSON. In lockstep
// mode the simulator waits for this program's answer to each step, so the run
// is deterministic.

/* <<<RSS>>>
robot shm_arm {
    body { shape: box; dimensions: [200mm, 200mm, 80mm]; mass: 5kg; }
    arm main {
        joints: 3;
        axes: [z, y, y];
        reach: 600mm;
        mass: 0.6kg;
        effort: 20Nm;
        speed: 120rpm;
    }
}
<<</RSS>>> */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "stacc_runtime.hpp"

int main() {
  try {
    stacc::Runtime rt;
    rt.load_file(__FILE__);
    // Servo gains (N·m/rad, N·m·s/rad): stiffer and less damped than the
    // runtime's defaults, so the joint follows a moving target closely.
    rt.gains(400, 8);
    stacc::ControlChannel ch(rt.enable_shm("lockstep", 1000));
    std::printf("channel: %u joints, %s\n", ch.n_joints(), ch.lockstep() ? "lockstep" : "realtime");

    stacc::ControlChannel::State s;
    if (!ch.wait_state(-1, s, 5.0)) throw std::runtime_error("no state from the runtime");

    const int steps = 3000;
    std::vector<double> rtt_ms;
    rtt_ms.reserve(steps);
    double worst_error = 0;
    std::vector<double> target(ch.n_joints(), 0.0);
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < steps; ++k) {
      // Base yaw follows a 0.5 Hz sine; shoulder and elbow hold a pose.
      target[0] = 0.6 * std::sin(2 * M_PI * 0.5 * s.sim_time);
      target[1] = 0.5;
      target[2] = -0.8;
      if (k > 1000) worst_error = std::max(worst_error, std::fabs(s.position[0] - target[0]));

      const auto a = std::chrono::steady_clock::now();
      ch.command_positions(target, s.step);
      const uint64_t answered = s.step;
      if (!ch.wait_state(static_cast<int64_t>(answered), s, 2.0)) throw std::runtime_error("runtime stopped publishing");
      rtt_ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count());
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::sort(rtt_ms.begin(), rtt_ms.end());
    auto status = rt.shm_status();
    std::printf("%d closed-loop steps in %.2f s: %.0f steps/s\n", steps, wall, steps / wall);
    std::printf("round trip ms: median %.3f  p99 %.3f  max %.3f\n", rtt_ms[steps / 2], rtt_ms[steps * 99 / 100], rtt_ms.back());
    std::printf("base joint tracking error (after settling): %.4f rad\n", worst_error);
    std::printf("missed deadlines %.0f, late commands %.0f\n", status["missed_deadlines"].num(0), status["late_commands"].num(0));
    rt.disable_shm();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  return 0;
}
