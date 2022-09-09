use core::ops::{Add, AddAssign, Mul, Sub};

use crate::Gains;

pub struct PidController<T: Default + Sub<Output = T> + Add + AddAssign + Copy, C: Mul<T> + Copy>
where
	C::Output: Into<C> + Add<Output = C::Output>,
{
	integral: T,
	derivative: T,
	previous_error: T,

	goal: T,

	gains: Gains<C>,
}

impl<T: Default + Sub<Output = T> + Add + AddAssign + Copy, C: Mul<T> + Copy> PidController<T, C>
where
	C::Output: Into<C> + Add<Output = C::Output>,
{
	pub fn new(goal: T, gains: Gains<C>) -> Self {
		Self {
			integral: Default::default(),
			derivative: Default::default(),
			previous_error: Default::default(),

			goal,

			gains,
		}
	}

	pub fn cycle(&mut self, current: T) -> C {
		let error = self.goal - current;

		self.integral += error;

		self.derivative = error - self.previous_error;

		self.previous_error = error;

		(self.gains.proportional * error
			+ self.gains.integral * self.integral
			+ self.gains.derivative * self.derivative)
			.into()
	}
}
