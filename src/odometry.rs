use core::time::Duration;

use uom::{
	si::f64::{Angle, Length, Ratio},
	ConstZero,
};
use vex_rt::{
	prelude::println,
	rotation::{RotationSensor, RotationSensorError},
};

use crate::{coordinates::Position, math::*};

pub struct OdometrySystem {
	left_sensor: RotationSensor,
	right_sensor: RotationSensor,
	rear_sensor: RotationSensor,

	wheel_radius: Length,

	last_left_angle: Angle,
	last_right_angle: Angle,
	last_rear_angle: Angle,

	left_wheel_offset: Length,
	right_wheel_offset: Length,
	rear_wheel_offset: Length,

	x_state: Length,
	y_state: Length,
	heading_state: Angle,
}

impl OdometrySystem {
	pub fn new(
		mut left_sensor: RotationSensor, mut right_sensor: RotationSensor, mut rear_sensor: RotationSensor,
		wheel_diameter: Length, left_wheel_offset: Length, right_wheel_offset: Length, rear_wheel_offset: Length,
	) -> Self {
		left_sensor.set_position(Angle::ZERO);
		right_sensor.set_position(Angle::ZERO);
		rear_sensor.set_position(Angle::ZERO);

		Self {
			left_sensor,
			right_sensor,
			rear_sensor,
			wheel_radius: wheel_diameter / 2.0,

			last_left_angle: Angle::ZERO,
			last_right_angle: Angle::ZERO,
			last_rear_angle: Angle::ZERO,

			left_wheel_offset,
			right_wheel_offset,
			rear_wheel_offset,

			x_state: Length::ZERO,
			y_state: Length::ZERO,
			heading_state: Angle::ZERO,
		}
	}

	pub fn get_position(&self) -> Position { Position::new(self.x_state, self.y_state, self.heading_state) }

	fn turn_diameter(&self) -> Length { self.left_wheel_offset + self.right_wheel_offset }

	pub fn cycle(&mut self) -> Result<(), RotationSensorError> {
		let left_angle: Angle = self.left_sensor.get_position()?;
		let right_angle: Angle = self.right_sensor.get_position()?;
		let rear_angle: Angle = self.rear_sensor.get_position()?;

		let left_distance_change: Length = (left_angle - self.last_left_angle) * self.wheel_radius;
		let right_distance_change: Length = (right_angle - self.last_right_angle) * self.wheel_radius;
		let rear_distance_change: Length = (rear_angle - self.last_rear_angle) * self.wheel_radius;

		let local_angle_change: Angle = ((left_distance_change - right_distance_change) / self.turn_diameter()).into();

		let coefficient: Ratio = 2.0 * (local_angle_change / 2.0).sin();

		let local_x_movement: Length = if local_angle_change == Angle::ZERO {
			rear_distance_change
		} else {
			coefficient * ((rear_distance_change / local_angle_change) + self.rear_wheel_offset)
		};

		let local_y_movement: Length = if local_angle_change == Angle::ZERO {
			right_distance_change
		} else {
			coefficient * ((right_distance_change / local_angle_change) + self.right_wheel_offset)
		};

		let average_angle: Angle = self.heading_state + (local_angle_change / 2.0);

		let polar_radius: Length = (local_x_movement * local_x_movement + local_y_movement * local_y_movement).sqrt();

		let polar_angle: Angle = local_y_movement.atan2(local_x_movement) - average_angle;

		let x_change: Length = polar_angle.cos() * polar_radius;
		let y_change: Length = polar_angle.sin() * polar_radius;

		self.last_left_angle = left_angle;
		self.last_right_angle = right_angle;
		self.last_rear_angle = rear_angle;

		let x: Length = self.x_state + x_change;
		let y: Length = self.y_state + y_change;
		let heading: Angle = self.heading_state + local_angle_change;

		self.x_state = x;
		self.y_state = y;
		self.heading_state = heading;

		Ok(())
	}
}
