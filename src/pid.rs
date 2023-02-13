use vex_rt::{
	prelude::println,
	rtos::{time_since_start, Instant},
};

use crate::Gains;

pub struct PositionController {
	integral: f64,
	previous_error: f64,
	previous_time: Instant,

	goal: f64,

	gains: Gains<f64>,

	end_threshold: f64,
}

impl PositionController {
	pub fn new(goal: f64, gains: Gains<f64>, end_threshold: f64) -> Self {
		Self {
			integral: 0.0,
			previous_error: 0.0,

			previous_time: time_since_start(),

			goal,

			gains,

			end_threshold,
		}
	}

	pub fn is_complete(&mut self, current: f64) -> bool {
		let error = self.goal - current;
		println!("{error} {}", self.end_threshold);
		error <= self.end_threshold && error >= -self.end_threshold
	}

	pub fn cycle(&mut self, current: f64) -> f64 {
		let error = self.goal - current;

		let delta_time = (time_since_start() - self.previous_time).as_secs_f64();

		self.integral += error * delta_time;

		let derivative: f64 = (error - self.previous_error) / delta_time;

		self.previous_error = error;

		self.previous_time = time_since_start();

		self.gains.proportional * error + self.gains.integral * self.integral + self.gains.derivative * derivative
	}
}

pub struct VelocityController {
	previous_error: f64,
	previous_previous_error: f64,
	previous_output: f64,
	previous_time: Instant,

	goal: f64,

	gains: Gains<f64>,

	target_threshold: f64,
}

impl VelocityController {
	pub fn new(goal: f64, gains: Gains<f64>, target_threshold: f64) -> Self {
		Self {
			previous_error: 0.0,
			previous_previous_error: 0.0,
			previous_output: 0.0,

			previous_time: time_since_start(),

			goal,

			gains,

			target_threshold,
		}
	}

	pub fn is_at_speed(&mut self, current: f64) -> bool {
		let error = self.goal - current;
		error <= self.target_threshold && error >= -self.target_threshold
	}

	pub fn cycle(&mut self, current: f64) -> f64 {
		let error = self.goal - current;
		let delta_error = error - self.previous_error;

		let delta_time = (time_since_start() - self.previous_time).as_secs_f64();

		let integral = error * delta_time;
		let derivative: f64 = (error - self.previous_error * 2.0 + self.previous_previous_error) / delta_time;

		self.previous_previous_error = self.previous_error;
		self.previous_error = error;

		self.previous_time = time_since_start();

		let output = self.previous_output
			+ self.gains.proportional * delta_error
			+ self.gains.integral * integral
			+ self.gains.derivative * derivative;

		self.previous_output = output;

		output
	}
}
