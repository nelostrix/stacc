// Arm reach: the body is described in RSS, the behaviour in C++.
//
//   g++ -std=c++17 -I bindings/cpp examples/embedded/arm_reach.cpp -o arm_reach && ./arm_reach
//
// load_file(__FILE__) sends this file to the STACC daemon, which reads the
// RSS block below and builds the arm in MuJoCo.

/* <<<RSS>>>
robot reach_arm {
    body {
        shape: box;
        dimensions: [200mm, 200mm, 80mm];
        mass: 5kg;
        material: steel;
    }

    arm main {
        joints: 3;
        axes: [z, y, y];
        reach: 600mm;
        mass: 0.6kg;
        material: aluminum;
    }
}
<<</RSS>>> */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "stacc_runtime.hpp"

int main() {
  stacc::Runtime rt;
  try {
    rt.load_file(__FILE__);
    rt.start();

    // Swing the base round and lift the shoulder and elbow.
    const double target[] = {0.6, 0.7, -0.9};
    rt.command_positions({target[0], target[1], target[2]});
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::printf("%-20s %10s %10s %10s\n", "joint", "target", "reached", "torque");
    int k = 0;
    for (const auto& j : rt.joints()) {
      std::printf("%-20s %9.1f° %9.1f° %8.2f N·m\n", j.name.c_str(), target[k] * 180.0 / M_PI,
                  j.position * 180.0 / M_PI, j.torque);
      ++k;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  return 0;
}
