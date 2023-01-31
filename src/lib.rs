#![no_std]

use core::time::Duration;

extern crate alloc;

pub mod controller;
pub mod motor;
pub mod pid;
pub mod tank_drive;
pub mod x_drive;

pub(crate) const PID_CYCLE_DURATION: Duration = Duration::from_millis(50);

/// Gains struct containing ratios for
#[derive(Clone, Copy)]
pub struct Gains<T> {
	pub proportional: T,
	pub integral: T,
	pub derivative: T,
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
macro_rules! ratio_gains {
	($kp:expr, $ki:expr, $kd:expr) => {
		$crate::Gains {
			proportional: $crate::ratio!($kp),
			integral: $crate::ratio!($ki),
			derivative: $crate::ratio!($kd),
		}
	};
}

#[macro_export]
macro_rules! rpm_gains {
	($kp:expr, $ki:expr, $kd:expr) => {
		$crate::Gains {
			proportional: $crate::rpm!($kp),
			integral: $crate::rpm!($ki),
			derivative: $crate::rpm!($kd),
		}
	};
}

#[macro_export]
macro_rules! gains {
	($kp:expr, $ki:expr, $kd:expr) => {
		$crate::Gains {
			proportional: $kp,
			integral: $ki,
			derivative: $kd,
		}
	};
}
