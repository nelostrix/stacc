//! Arm over shared memory: a lockstep control loop in Rust.
//!
//!     cargo run --release --example arm_shm
//!
//! HTTP loads this file's robot and enables the channel; every control step
//! then goes through /dev/shm.

/* <<<RSS>>>
robot shm_arm_rs {
    body { shape: box; dimensions: [200mm, 200mm, 80mm]; mass: 5kg; }
    arm main { joints: 3; axes: [z, y, y]; reach: 600mm; mass: 0.6kg; effort: 20Nm; speed: 120rpm; }
}
<<</RSS>>> */

use std::time::{Duration, Instant};

fn main() -> stacc_runtime::Result<()> {
    let mut rt = stacc_runtime::Runtime::default();
    let path = match option_env!("CARGO_MANIFEST_DIR") {
        Some(dir) => format!("{dir}/{}", file!()),
        None => file!().to_string(),
    };
    rt.load_file(&path)?;
    // Servo gains (N·m/rad, N·m·s/rad): stiffer and less damped than the
    // runtime's defaults, so the joint follows a moving target closely.
    rt.gains(400.0, 8.0)?;
    let mut ch = rt.enable_shm("lockstep", 1000.0)?;
    let mut s = ch.wait_state(None, Duration::from_secs(5))?.expect("no state from the runtime");

    let steps = 3000;
    let mut rtt_ms = Vec::with_capacity(steps);
    let mut worst_error: f64 = 0.0;
    let mut target = vec![0.0; ch.n_joints()];
    let t0 = Instant::now();
    for k in 0..steps {
        target[0] = 0.6 * (2.0 * std::f64::consts::PI * 0.5 * s.sim_time).sin();
        target[1] = 0.5;
        target[2] = -0.8;
        if k > 1000 {
            worst_error = worst_error.max((s.position[0] - target[0]).abs());
        }
        let a = Instant::now();
        ch.command_positions(&target, s.step, None)?;
        s = ch.wait_state(Some(s.step), Duration::from_secs(2))?.expect("runtime stopped publishing");
        rtt_ms.push(a.elapsed().as_secs_f64() * 1e3);
    }
    let wall = t0.elapsed().as_secs_f64();
    rtt_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let status = rt.shm_status()?;
    println!("{steps} closed-loop steps in {wall:.2} s: {:.0} steps/s", steps as f64 / wall);
    println!(
        "round trip ms: median {:.3}  p99 {:.3}  max {:.3}",
        rtt_ms[steps / 2],
        rtt_ms[steps * 99 / 100],
        rtt_ms[steps - 1]
    );
    println!("base joint tracking error (after settling): {worst_error:.4} rad");
    println!(
        "missed deadlines {}, late commands {}",
        status.get("missed_deadlines").as_f64().unwrap_or(0.0),
        status.get("late_commands").as_f64().unwrap_or(0.0)
    );
    rt.disable_shm()?;
    Ok(())
}
