#![no_std]

extern crate alloc;

use controller::Controller;
use motor::Motor;
use tank_drive::TankDrive;
use uom::si::{f64::Length, length::inch};
use vex_rt::{
	prelude::{Gearset, Peripherals},
	robot::Robot,
};

pub mod controller;
pub mod motor;
pub mod pid;
pub mod tank_drive;

/// Gains struct containing ratios for
#[derive(Clone, Copy)]
pub struct Gains<T> {
	proportional: T,
	integral: T,
	derivative: T,
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

struct Bot {
	controller: Controller,
	drive_train: TankDrive<2>,
}

impl Robot for Bot {
	fn new(peripherals: Peripherals) -> Self {
		Self {
			controller: peripherals.master_controller.into(),
			drive_train: TankDrive::new(
				[
					Motor::new(peripherals.port01, Gearset::EighteenToOne, true),
					Motor::new(peripherals.port02, Gearset::EighteenToOne, false),
				],
				[
					Motor::new(peripherals.port03, Gearset::EighteenToOne, true),
					Motor::new(peripherals.port04, Gearset::EighteenToOne, false),
				],
				ratio!(1.0),
				Length::new::<inch>(4.0),
				Length::new::<inch>(19.0),
				Length::new::<inch>(19.0),
				ratio_gains!(0.0, 0.0, 0.0),
				ratio_gains!(0.0, 0.0, 0.0),
			),
		}
	}

	fn opcontrol(&'static self, _ctx: vex_rt::rtos::Context) {
		self.drive_train.drive_tank(
			self.controller.left_stick().get_y(),
			self.controller.right_stick().get_y(),
		)
	}
}
