use uom::{
	si::f64::{Angle, AngularVelocity, ElectricPotential, Length, Ratio},
	ConstZero,
};
use vex_rt::{
	prelude::{Motor, MotorError},
	rtos::{Context, Loop},
	select,
};

use crate::{
	pid::{PositionController, VelocityController},
	Gains,
	PID_CYCLE_DURATION,
};

pub struct TankDrive<const N: usize> {
	pub left_motors: [Motor; N],
	pub right_motors: [Motor; N],

	pub drive_ratio: Ratio,
	pub wheel_diameter: Length,
	pub track_width: Length,

	pub distance_gains: Gains,
	pub turn_gains: Gains,

	pub left_velocity_gains: Gains,
	pub right_velocity_gains: Gains,

	pub position_threshold: Angle,
	pub velocity_threshold: AngularVelocity,
}

impl<const N: usize> TankDrive<N> {
	fn wheel_radius(&self) -> Length { return self.wheel_diameter / 2.0; }

	/// Sets the drive train motor powers based on a left and right input,
	pub fn drive_tank(&mut self, left: Ratio, right: Ratio) -> Result<(), MotorError> {
		self.drive_left(left)?;
		self.drive_right(right)?;
		Ok(())
	}

	/// Sets the drive train motor powers based on a horizontal and rotational input
	pub fn drive_arcade(&mut self, x: Ratio, y: Ratio) -> Result<(), MotorError> {
		self.drive_left(y + x)?;
		self.drive_right(y - x)?;
		Ok(())
	}

	fn drive_left(&mut self, value: Ratio) -> Result<(), MotorError> {
		for motor in self.left_motors.iter_mut() {
			motor.move_ratio(value)?
		}
		Ok(())
	}

	fn drive_left_voltage(&mut self, voltage: ElectricPotential) -> Result<(), MotorError> {
		for motor in self.left_motors.iter_mut() {
			motor.move_voltage(voltage)?
		}
		Ok(())
	}

	fn drive_right(&mut self, value: Ratio) -> Result<(), MotorError> {
		for motor in self.right_motors.iter_mut() {
			motor.move_ratio(value)?
		}
		Ok(())
	}

	fn drive_right_voltage(&mut self, voltage: ElectricPotential) -> Result<(), MotorError> {
		for motor in self.right_motors.iter_mut() {
			motor.move_voltage(voltage)?
		}
		Ok(())
	}

	fn tare_left_postition(&mut self) -> Result<(), MotorError> { self.left_motors[0].tare_position() }

	fn tare_right_postition(&mut self) -> Result<(), MotorError> { self.right_motors[0].tare_position() }

	fn get_left_position(&self) -> Result<Angle, MotorError> { self.left_motors[0].get_position() }

	fn get_right_position(&self) -> Result<Angle, MotorError> { self.right_motors[0].get_position() }

	fn get_left_velocity(&self) -> Result<AngularVelocity, MotorError> { self.left_motors[0].get_actual_velocity() }

	fn get_right_velocity(&self) -> Result<AngularVelocity, MotorError> { self.right_motors[0].get_actual_velocity() }

	/// Moves the drive train a specified relative distance
	pub fn drive_distance(&mut self, distance: Length, ctx: &Context) -> Result<(), MotorError> {
		self.tare_left_postition()?;
		self.tare_right_postition()?;

		let wheel_rotation_goal: Angle = (distance / self.wheel_radius()).into();

		let motor_rotation_goal: Angle = (wheel_rotation_goal / self.drive_ratio).into();

		let mut left_position_controller =
			PositionController::new(motor_rotation_goal, self.distance_gains, self.position_threshold);
		let mut right_position_controller =
			PositionController::new(motor_rotation_goal, self.distance_gains, self.position_threshold);

		let mut left_speed_controller =
			VelocityController::new(AngularVelocity::ZERO, self.left_velocity_gains, self.velocity_threshold);
		let mut right_speed_controller = VelocityController::new(
			AngularVelocity::ZERO,
			self.right_velocity_gains,
			self.velocity_threshold,
		);

		let mut pause = Loop::new(PID_CYCLE_DURATION);

		loop {
			if left_position_controller.is_complete(self.get_left_position()?)
				&& right_position_controller.is_complete(self.get_right_position()?)
			{
				self.drive_left(Ratio::ZERO)?;
				self.drive_right(Ratio::ZERO)?;
				break;
			}

			let left_motor_speed: AngularVelocity = left_position_controller.cycle(self.get_left_position()?);
			let right_motor_speed: AngularVelocity = right_position_controller.cycle(self.get_right_position()?);

			left_speed_controller.set_target(left_motor_speed);
			right_speed_controller.set_target(right_motor_speed);

			let left_motor_voltage: ElectricPotential = left_speed_controller.cycle(self.get_left_velocity()?);
			let right_motor_voltage: ElectricPotential = right_speed_controller.cycle(self.get_right_velocity()?);

			self.drive_left_voltage(left_motor_voltage)?;
			self.drive_right_voltage(right_motor_voltage)?;

			select! {
				_ = ctx.done() => break,
				_ = pause.select() => continue
			}
		}

		Ok(())
	}

	/// Rotates the drive train a specified relative angle
	pub fn rotate_angle(&mut self, angle: Angle, ctx: &Context) -> Result<(), MotorError> {
		self.tare_left_postition()?;
		self.tare_right_postition()?;

		let wheel_rotation_goal: Angle = ((angle * self.track_width * 0.5) / self.wheel_radius()).into();

		let motor_rotation_goal: Angle = (wheel_rotation_goal / self.drive_ratio).into();

		let mut left_position_controller =
			PositionController::new(motor_rotation_goal, self.turn_gains, self.position_threshold);
		let mut right_position_controller =
			PositionController::new(-motor_rotation_goal, self.turn_gains, self.position_threshold);

		let mut left_speed_controller =
			VelocityController::new(AngularVelocity::ZERO, self.left_velocity_gains, self.velocity_threshold);
		let mut right_speed_controller = VelocityController::new(
			AngularVelocity::ZERO,
			self.right_velocity_gains,
			self.velocity_threshold,
		);

		let mut pause = Loop::new(PID_CYCLE_DURATION);

		loop {
			if left_position_controller.is_complete(self.get_left_position()?)
				&& right_position_controller.is_complete(self.get_right_position()?)
			{
				self.drive_left(Ratio::ZERO)?;
				self.drive_right(Ratio::ZERO)?;
				break;
			}

			let left_motor_speed: AngularVelocity = left_position_controller.cycle(self.get_left_position()?);
			let right_motor_speed: AngularVelocity = right_position_controller.cycle(self.get_right_position()?);

			left_speed_controller.set_target(left_motor_speed);
			right_speed_controller.set_target(right_motor_speed);

			let left_motor_voltage: ElectricPotential = left_speed_controller.cycle(self.get_left_velocity()?);
			let right_motor_voltage: ElectricPotential = right_speed_controller.cycle(self.get_right_velocity()?);

			self.drive_left_voltage(left_motor_voltage)?;
			self.drive_right_voltage(right_motor_voltage)?;
			select! {
				_ = ctx.done() => break,
				_ = pause.select() => continue
			}
		}

		Ok(())
	}
}
