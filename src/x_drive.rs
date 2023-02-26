use uom::si::f64::{Length, Ratio};
use vex_rt::prelude::{Motor, MotorError};

pub struct XDrive {
	front_left_motor: Motor,
	back_left_motor: Motor,
	front_right_motor: Motor,
	back_right_motor: Motor,

	drive_ratio: Ratio,
	wheel_radius: Length,

	turn_radius: Length,
}

impl XDrive {
	pub fn new(
		front_left_motor: Motor, back_left_motor: Motor, front_right_motor: Motor, back_right_motor: Motor,
		drive_ratio: Ratio, wheel_diameter: Length, turn_diameter: Length,
	) -> Self {
		Self {
			front_left_motor,
			back_left_motor,
			front_right_motor,
			back_right_motor,
			drive_ratio,
			wheel_radius: wheel_diameter / 2.0,
			turn_radius: turn_diameter / 2.0,
		}
	}

	pub fn drive(&mut self, x: Ratio, y: Ratio, rotation: Ratio) -> Result<(), MotorError> {
		self.front_left_motor.move_ratio(y + x + rotation)?;
		self.front_right_motor.move_ratio(y - x - rotation)?;
		self.back_left_motor.move_ratio(y - x + rotation)?;
		self.back_right_motor.move_ratio(y + x - rotation)?;
		Ok(())
	}
}
