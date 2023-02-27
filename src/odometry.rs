use uom::si::{
	angle::radian,
	f64::{Angle, Length, Ratio},
	ratio::ratio,
};
use vex_rt::rotation::{RotationSensor, RotationSensorError};
pub struct Position {
	x: Length,
	y: Length,
	heading: Angle,
}

pub struct OdometrySystem {
	left_sensor: RotationSensor,
	right_sensor: RotationSensor,
	rear_sensor: RotationSensor,

	wheel_diameter: Length,

	last_left_angle: Angle,
	last_right_angle: Angle,
	last_rear_angle: Angle,

	left_wheel_offset: Length,
	right_wheel_offset: Length,
	rear_wheel_offset: Length,

	last_x: Length,
	last_y: Length,
	last_heading: Angle,
}

impl OdometrySystem {
	pub fn wheel_radius(&self) -> Length { self.wheel_diameter / 2.0 }

	pub fn turn_diameter(&self) -> Length { self.left_wheel_offset + self.right_wheel_offset }

	pub fn cycle(&mut self) -> Result<Position, RotationSensorError> {
		let left_angle: Angle = self.left_sensor.get_angle()?;
		let right_angle: Angle = self.right_sensor.get_angle()?;
		let rear_angle: Angle = self.rear_sensor.get_angle()?;

		let left_distance_change: Length = (self.last_left_angle - left_angle) * self.wheel_radius();
		let right_distance_change: Length = (self.last_right_angle - right_angle) * self.wheel_radius();
		let rear_distance_change: Length = (self.last_rear_angle - rear_angle) * self.wheel_radius();

		let arc_angle: Angle = ((left_distance_change - right_distance_change) / self.turn_diameter()).into();

		let coefficient: Ratio = 2.0 * Ratio::new::<ratio>(libm::sin((arc_angle / 2.0).get::<radian>())); // No STD lib for trig functions

		let x_movement: Length = coefficient * ((rear_distance_change / arc_angle) + self.rear_wheel_offset);
		let y_movement: Length = coefficient * ((right_distance_change / arc_angle) + self.right_wheel_offset);

		let second_coefficient: Ratio =
			Ratio::new::<ratio>(libm::cos((self.last_heading + (arc_angle / 2.0)).get::<radian>()));

		let x_change: Length = second_coefficient * x_movement;
		let y_change: Length = second_coefficient * y_movement;

		self.last_left_angle = left_angle;
		self.last_right_angle = right_angle;
		self.last_rear_angle = rear_angle;

		let x: Length = self.last_x + x_change;
		let y: Length = self.last_y + y_change;
		let heading: Angle = self.last_heading + arc_angle;

		self.last_x = x;
		self.last_y = y;
		self.last_heading = heading;

		Ok(Position { x, y, heading })
	}
}
