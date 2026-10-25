//! Quadruped crouch: the body is described in RSS, the behaviour in Rust.
//!
//!     cargo run --example quadruped_crouch
//!
//! `load_file(file!())` sends this file to the STACC daemon, which reads the
//! RSS block below and builds the quadruped in MuJoCo.

/* <<<RSS>>>
robot pup {
    body {
        shape: box;
        dimensions: [450mm, 220mm, 100mm];
        mass: 3kg;
        material: carbon_fiber;
    }

    leg front_left  { joints: 3; reach: 300mm; mass: 0.2kg; }
    leg front_right { joints: 3; reach: 300mm; mass: 0.2kg; }
    leg rear_left   { joints: 3; reach: 300mm; mass: 0.2kg; }
    leg rear_right  { joints: 3; reach: 300mm; mass: 0.2kg; }

    sensors {
        imu body { model: bmi088; }
    }
}
<<</RSS>>> */

use std::{thread, time::Duration};

fn main() -> stacc_runtime::Result<()> {
    let mut rt = stacc_runtime::Runtime::default();
    // Under cargo, file!() is relative to the crate root; under plain rustc
    // (the Studio's Run button) it is the path rustc was given.
    let path = match option_env!("CARGO_MANIFEST_DIR") {
        Some(dir) => format!("{dir}/{}", file!()),
        None => file!().to_string(),
    };
    rt.load_file(&path)?;
    rt.start()?;
    thread::sleep(Duration::from_secs(1));
    let standing = rt.pose()?[2];

    // Hip flexion +30°, knee -60° on every leg: the body lowers but stays level.
    let joints = rt.joints()?;
    let crouch: Vec<f64> = joints
        .iter()
        .map(|j| match &j.name {
            n if n.ends_with("_2_joint") => 30f64.to_radians(),
            n if n.ends_with("_3_joint") => (-60f64).to_radians(),
            _ => 0.0,
        })
        .collect();
    // Ramp the setpoints over a second instead of stepping them: a step on
    // light legs makes the servos kick the body off the ground.
    const STEPS: u32 = 50;
    for k in 1..=STEPS {
        let s = f64::from(k) / f64::from(STEPS);
        let blend = s * s * (3.0 - 2.0 * s); // smoothstep: zero velocity at both ends
        let setpoint: Vec<f64> = crouch.iter().map(|q| q * blend).collect();
        rt.command_positions(&setpoint)?;
        thread::sleep(Duration::from_millis(20));
    }
    thread::sleep(Duration::from_secs(1));

    let pose = rt.pose()?;
    println!("body height: standing {standing:.3} m -> crouched {:.3} m", pose[2]);
    println!("tilt: roll {:+.1}°, pitch {:+.1}°", pose[3].to_degrees(), pose[4].to_degrees());
    for j in rt.joints()?.iter().filter(|j| j.name.ends_with("_3_joint")) {
        println!("{:24} {:+7.1}°  {:+6.2} N·m", j.name, j.position.to_degrees(), j.torque);
    }
    Ok(())
}
