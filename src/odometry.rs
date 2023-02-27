use core::time::Duration;

use uom::{
	si::{
		angle::radian,
		f64::{Angle, Length, Ratio},
		length::meter,
	},
	ConstZero,
};
use vex_rt::{
	rotation::{RotationSensor, RotationSensorError},
	rtos::{Loop, Task},
};

use crate::math::*;

pub struct Position {
	x: Length,
	y: Length,
	heading: Angle,
}

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
		left_sensor: RotationSensor, right_sensor: RotationSensor, rear_sensor: RotationSensor, wheel_diameter: Length,
		left_wheel_offset: Length, right_wheel_offset: Length, rear_wheel_offset: Length,
	) -> Self {
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

	pub fn start_tracking(&'static mut self) {
		let mut cycle = Loop::new(Duration::from_millis(20));
		Task::spawn(move || loop {
			self.cycle().unwrap();
			cycle.delay();
		})
		.unwrap();
	}

	pub fn get_position(&self) -> Position {
		Position {
			x: self.x_state,
			y: self.y_state,
			heading: self.heading_state,
		}
	}

	fn turn_diameter(&self) -> Length { self.left_wheel_offset + self.right_wheel_offset }

	fn cycle(&mut self) -> Result<(), RotationSensorError> {
		let left_angle: Angle = self.left_sensor.get_angle()?;
		let right_angle: Angle = self.right_sensor.get_angle()?;
		let rear_angle: Angle = self.rear_sensor.get_angle()?;

		let left_distance_change: Length = (self.last_left_angle - left_angle) * self.wheel_radius;
		let right_distance_change: Length = (self.last_right_angle - right_angle) * self.wheel_radius;
		let rear_distance_change: Length = (self.last_rear_angle - rear_angle) * self.wheel_radius;

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

		let x_change: Length = polar_angle.sin() * polar_radius;
		let y_change: Length = polar_angle.cos() * polar_radius;

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
