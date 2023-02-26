use uom::{
	si::{
		angular_acceleration::radian_per_second_squared,
		electric_potential::volt,
		f64::{Angle, AngularAbsement, AngularAcceleration, AngularVelocity, ElectricPotential, Time},
	},
	ConstZero,
};
use vex_rt::rtos::{time_since_start, Instant};

use crate::Gains;

pub struct PositionController {
	integral: AngularAbsement,
	previous_error: Option<Angle>,
	previous_time: Instant,

	target: Angle,

	gains: Gains,

	completion_threshold: Angle,
}

impl PositionController {
	pub fn new(target: Angle, gains: Gains, completion_threshold: Angle) -> Self {
		Self {
			integral: AngularAbsement::ZERO,
			previous_error: None,

			previous_time: time_since_start(),

			target,

			gains,

			completion_threshold,
		}
	}

	/// Determines whether the controller has reached the target by checking if the error has changed since the last
	/// cycle
	pub fn is_complete(&self, current: Angle) -> bool {
		self.previous_error
			.map(|previous_error: Angle| (current - self.target) == previous_error)
			.unwrap_or(false)
	}

	/// Sets the current target that the controller is aiming for
	pub fn set_target(&mut self, target: Angle) { self.target = target; }

	/// Runs a cycle of the PID and returns the velocity the controller determines the motors should be moving at
	pub fn cycle(&mut self, current: Angle) -> AngularVelocity {
		let error: Angle = self.target - current;

		let delta_time: Time = (time_since_start() - self.previous_time).try_into().unwrap();

		self.integral += (error * delta_time).into();

		let previous_error: Angle = self.previous_error.unwrap_or(error);

		let derivative: AngularVelocity = ((error - previous_error) / delta_time).into();

		self.previous_error = Some(error);

		self.previous_time = time_since_start();

		let proportional_channel: AngularVelocity = (self.gains.proportional * error).into();

		let integral_channel: AngularVelocity = (self.gains.integral * self.integral).into();

		let derivative_channel: AngularVelocity = (self.gains.derivative * derivative).into();

		proportional_channel + integral_channel + derivative_channel
	}
}

pub struct VelocityController {
	previous_error: Option<AngularVelocity>,
	previous_previous_error: Option<AngularVelocity>,
	previous_output: AngularAcceleration,
	previous_time: Instant,

	target: AngularVelocity,

	gains: Gains,

	target_threshold: AngularVelocity,
}

impl VelocityController {
	pub fn new(target: AngularVelocity, gains: Gains, target_threshold: AngularVelocity) -> Self {
		Self {
			previous_error: None,
			previous_previous_error: None,
			previous_output: AngularAcceleration::ZERO,

			previous_time: time_since_start(),

			target,

			gains,

			target_threshold,
		}
	}

	pub fn is_at_speed(&self, current: AngularVelocity) -> bool {
		let error: AngularVelocity = self.target - current;
		error.abs() <= self.target_threshold
	}

	pub fn set_target(&mut self, target: AngularVelocity) { self.target = target; }

	pub fn cycle(&mut self, current: AngularVelocity) -> ElectricPotential {
		let error: AngularVelocity = self.target - current;

		let previous_error: AngularVelocity = self.previous_error.unwrap_or(error);
		let previous_previous_error: AngularVelocity = self.previous_previous_error.unwrap_or(error);

		let delta_error: AngularVelocity = error - previous_error;

		let delta_time: Time = (time_since_start() - self.previous_time).try_into().unwrap();

		let integral: Angle = (error * delta_time).into();
		let derivative: AngularAcceleration =
			((error - previous_error * 2.0 + previous_previous_error) / delta_time).into();

		self.previous_previous_error = Some(previous_error);
		self.previous_error = Some(error);

		self.previous_time = time_since_start();

		let proportional_channel: AngularAcceleration = (self.gains.proportional * delta_error).into();
		let integral_channel: AngularAcceleration = (self.gains.integral * integral).into();
		let derivative_channel: AngularAcceleration = (self.gains.derivative * derivative).into();

		let output: AngularAcceleration =
			self.previous_output + proportional_channel + integral_channel + derivative_channel;

		self.previous_output = output;

		ElectricPotential::new::<volt>(
			(proportional_channel + integral_channel + derivative_channel).get::<radian_per_second_squared>(),
		)
	}
}
