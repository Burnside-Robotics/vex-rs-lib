use uom::{
	si::{
		angle::radian,
		f64::{Angle, Length, Ratio},
		ratio::ratio,
	},
	ConstZero,
};
use vex_rt::{
	macros,
	prelude::println,
	rtos::{Context, Loop},
	select,
};

use crate::{motor::Motor, pid::PidController, Gains, PID_CYCLE_DURATION};

pub struct TankDrive<const N: usize> {
	left_motors: [Motor; N],
	right_motors: [Motor; N],

	drive_ratio: Ratio,
	wheel_radius: Length,

	track_width: Length,

	distance_gains: Gains<f64>,
	turn_gains: Gains<f64>,

	pid_end_threshold: Angle,
}

impl<const N: usize> TankDrive<N> {
	pub fn new(
		left_motors: [Motor; N], right_motors: [Motor; N], drive_ratio: Ratio, wheel_diameter: Length,
		track_width: Length, track_length: Length, distance_gains: Gains<f64>, turn_gains: Gains<f64>,
		pid_end_threshold: Angle,
	) -> Self {
		Self {
			left_motors,
			right_motors,
			drive_ratio,
			wheel_radius: wheel_diameter / 2.0,
			track_width,
			distance_gains,
			turn_gains,
			pid_end_threshold,
		}
	}

	pub fn drive_tank(&mut self, left: Ratio, right: Ratio) {
		self.drive_left(left);
		self.drive_right(right);
	}

	pub fn drive_arcade(&mut self, x: Ratio, y: Ratio) {
		self.drive_left(y + x);
		self.drive_right(y - x);
	}

	fn drive_left(&mut self, value: Ratio) {
		for i in 0..N {
			self.left_motors[i].move_ratio(value);
		}
	}

	fn drive_right(&mut self, value: Ratio) {
		for i in 0..N {
			self.right_motors[i].move_ratio(value);
		}
	}

	fn tare_left_postition(&mut self) { self.left_motors[0].tare_position(); }

	fn tare_right_postition(&mut self) { self.right_motors[0].tare_position(); }

	fn get_left_position(&self) -> Angle { self.left_motors[0].get_position() }

	fn get_right_position(&self) -> Angle { self.right_motors[0].get_position() }

	pub fn drive_distance(&mut self, distance: Length, ctx: &Context) {
		self.tare_left_postition();
		self.tare_right_postition();

		let wheel_rotation_goal = (distance / self.wheel_radius).get::<ratio>();

		let motor_rotation_goal = (wheel_rotation_goal / self.drive_ratio).get::<ratio>();

		let mut left_controller = PidController::new(
			motor_rotation_goal,
			self.distance_gains,
			PID_CYCLE_DURATION,
			self.pid_end_threshold.get::<radian>(),
		);
		let mut right_controller = PidController::new(
			motor_rotation_goal,
			self.distance_gains,
			PID_CYCLE_DURATION,
			self.pid_end_threshold.get::<radian>(),
		);

		let mut pause = Loop::new(PID_CYCLE_DURATION);

		loop {
			if left_controller.is_complete(self.get_left_position().get::<radian>())
				&& right_controller.is_complete(self.get_right_position().get::<radian>())
			{
				self.drive_left(Ratio::ZERO);
				self.drive_right(Ratio::ZERO);
				break;
			}

			let left_motor_speed = left_controller.cycle(self.get_left_position().get::<radian>());
			let right_motor_speed = right_controller.cycle(self.get_right_position().get::<radian>());

			self.drive_left(Ratio::new::<ratio>(left_motor_speed));
			self.drive_right(Ratio::new::<ratio>(right_motor_speed));

			select! {
				_ = ctx.done() => break,
				_ = pause.select() => continue
			}
		}
	}

	pub fn rotate_angle(&mut self, angle: Angle, ctx: &Context) {
		self.tare_left_postition();
		self.tare_right_postition();

		let wheel_rotation_goal = ((angle * self.track_width * 0.5) / self.wheel_radius).get::<ratio>();

		let motor_rotation_goal = (wheel_rotation_goal / self.drive_ratio).get::<ratio>();

		let mut left_controller = PidController::new(
			motor_rotation_goal,
			self.turn_gains,
			PID_CYCLE_DURATION,
			self.pid_end_threshold.get::<radian>(),
		);
		let mut right_controller = PidController::new(
			-motor_rotation_goal,
			self.turn_gains,
			PID_CYCLE_DURATION,
			self.pid_end_threshold.get::<radian>(),
		);

		let mut pause = Loop::new(PID_CYCLE_DURATION);

		loop {
			if left_controller.is_complete(self.get_left_position().get::<radian>())
				&& right_controller.is_complete(self.get_right_position().get::<radian>())
			{
				println!("done");
				self.drive_left(Ratio::ZERO);
				self.drive_right(Ratio::ZERO);
				break;
			}

			let left_motor_speed = left_controller.cycle(self.get_left_position().get::<radian>());
			let right_motor_speed = right_controller.cycle(self.get_right_position().get::<radian>());

			self.drive_left(Ratio::new::<ratio>(left_motor_speed));
			self.drive_right(Ratio::new::<ratio>(right_motor_speed));

			select! {
				_ = ctx.done() => break,
				_ = pause.select() => continue
			}
		}
	}
}
