use libm::{pow, sqrt};
use uom::si::{
	f64::{Angle, Length, Ratio},
	length::meter,
};

use crate::{motor::Motor, pid::PidController, Gains};

pub struct TankDrive<const N: usize> {
	left_motors: [Motor; N],
	right_motors: [Motor; N],

	drive_ratio: Ratio,
	wheel_radius: Length,

	turn_radius: Length,

	distance_gains: Gains<Ratio>,
	turn_gains: Gains<Ratio>,
}

impl<const N: usize> TankDrive<N> {
	pub fn new(
		left_motors: [Motor; N], right_motors: [Motor; N], drive_ratio: Ratio, wheel_diameter: Length,
		track_width: Length, track_length: Length, distance_gains: Gains<Ratio>, turn_gains: Gains<Ratio>,
	) -> Self {
		Self {
			left_motors,
			right_motors,
			drive_ratio,
			wheel_radius: wheel_diameter / 2.0,
			turn_radius: Length::new::<meter>(
				sqrt(pow(track_length.get::<meter>(), 2.0) + pow(track_width.get::<meter>(), 2.0)) / 2.0,
			),
			distance_gains,
			turn_gains,
		}
	}

	pub fn drive_tank(&self, left: Ratio, right: Ratio) {
		self.drive_left(left);
		self.drive_right(right);
	}

	pub fn drive_arcade(&self, x: Ratio, y: Ratio) {
		self.drive_left(y + x);
		self.drive_right(y - x);
	}

	fn drive_left(&self, value: Ratio) {
		for i in 0..N {
			self.left_motors[i].move_ratio(value);
		}
	}

	fn drive_right(&self, value: Ratio) {
		for i in 0..N {
			self.right_motors[i].move_ratio(value);
		}
	}

	fn tare_left_postition(&self) { self.left_motors[0].tare_position(); }

	fn tare_right_postition(&self) { self.right_motors[0].tare_position(); }

	fn get_left_position(&self) -> Angle { self.left_motors[0].get_position() }

	fn get_right_position(&self) -> Angle { self.right_motors[0].get_position() }

	pub fn drive_distance(&self, distance: Length) {
		self.tare_left_postition();
		self.tare_right_postition();

		let wheel_rotation_goal: Angle = (distance / self.wheel_radius).into();

		let motor_rotation_goal: Angle = (wheel_rotation_goal / self.drive_ratio).into();

		let mut left_controller: PidController<Angle, Ratio> =
			PidController::new(motor_rotation_goal, self.distance_gains);
		let mut right_controller: PidController<Angle, Ratio> =
			PidController::new(motor_rotation_goal, self.distance_gains);

		loop {
			let left_motor_speed: Ratio = left_controller.cycle(self.get_left_position());
			let right_motor_speed: Ratio = right_controller.cycle(self.get_right_position());

			self.drive_left(left_motor_speed);
			self.drive_right(right_motor_speed);
		}
	}

	pub fn rotate_angle(&self, angle: Angle) {
		self.tare_left_postition();
		self.tare_right_postition();

		let rotation_goal: Angle = ((angle * self.turn_radius) / self.wheel_radius).into();

		let mut left_controller: PidController<Angle, Ratio> = PidController::new(rotation_goal, self.turn_gains);
		let mut right_controller: PidController<Angle, Ratio> = PidController::new(-rotation_goal, self.turn_gains);

		loop {
			let left_motor_speed: Ratio = left_controller.cycle(self.get_left_position());
			let right_motor_speed: Ratio = right_controller.cycle(self.get_right_position());

			self.drive_left(left_motor_speed);
			self.drive_right(right_motor_speed);
		}
	}
}
