//! Library with utilities for vex rust robots
#![no_std]
#![warn(missing_docs)]

use core::time::Duration;

use uom::si::f64::{Frequency, FrequencyDrift, Ratio};

extern crate alloc;

pub mod coordinates;
mod math;
pub mod odometry;
pub mod pid;
pub mod pure_pursuit;
pub mod tank_drive;
pub mod x_drive;

/// Constant for
pub(crate) const PID_CYCLE_DURATION: Duration = Duration::from_millis(50);

/// Gains struct containing ratios for
#[derive(Clone, Copy)]
pub struct Gains<P = Frequency, I = FrequencyDrift, D = Ratio> {
	pub proportional: P,
	pub integral: I,
	pub derivative: D,
}

#[macro_export]
macro_rules! ratio {
	($value:expr) => {
		uom::si::f64::Ratio::new::<uom::si::ratio::ratio>($value)
	};
}

#[macro_export]
macro_rules! rpm {
	($value:expr) => {
		uom::si::f64::AngularVelocity::new::<uom::si::angular_velocity::revolution_per_minute>($value)
	};
}

#[macro_export]
macro_rules! hertz {
	($value:expr) => {
		uom::si::f64::Frequency::new::<uom::si::frequency::hertz>($value)
	};
}

#[macro_export]
macro_rules! hertz_per_second {
	($value:expr) => {
		uom::si::f64::FrequencyDrift::new::<uom::si::frequency_drift::hertz_per_second>($value)
	};
}

#[macro_export]
macro_rules! gains {
	($kp:expr, $ki:expr, $kd:expr) => {
		$crate::Gains {
			proportional: $crate::hertz!($kp),
			integral: $crate::hertz_per_second!($ki),
			derivative: $crate::ratio!($kd),
		}
	};
}
