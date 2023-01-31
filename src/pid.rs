use core::time::Duration;

use vex_rt::rtos::{time_since_start, Instant};

use crate::Gains;

pub struct PidController {
	integral: f64,
	derivative: f64,
	previous_error: f64,
	previous_time: Instant,

	expected_cycle_time: Duration,

	goal: f64,

	gains: Gains<f64>,

	end_threshold: f64,
}

impl PidController {
	pub fn new(goal: f64, gains: Gains<f64>, expected_cycle_time: Duration, end_threshold: f64) -> Self {
		Self {
			integral: Default::default(),
			derivative: Default::default(),
			previous_error: Default::default(),

			previous_time: time_since_start(),

			expected_cycle_time,

			goal,

			gains,

			end_threshold,
		}
	}

	pub fn is_complete(&mut self, current: f64) -> bool {
		let error = self.goal - current;
		error <= self.end_threshold && error >= -self.end_threshold
	}

	pub fn cycle(&mut self, current: f64) -> f64 {
		let error = self.goal - current;

		let delta_time =
			(time_since_start() - self.previous_time).as_secs_f64() / self.expected_cycle_time.as_secs_f64();

		self.integral += error * delta_time;

		self.derivative = (error - self.previous_error) / delta_time;

		self.previous_error = error;

		self.previous_time = time_since_start();

		self.gains.proportional * error + self.gains.integral * self.integral + self.gains.derivative * self.derivative
	}
}
