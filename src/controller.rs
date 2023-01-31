use uom::si::{f64::Ratio, ratio::ratio};

pub struct AnalogStick<'a> {
	internal: &'a vex_rt::controller::AnalogStick,
}

impl<'a> AnalogStick<'a> {
	pub fn get_x(&self) -> Ratio { Ratio::new::<ratio>(self.internal.get_x().unwrap() as f64 / 127.0) }

	pub fn get_y(&self) -> Ratio { Ratio::new::<ratio>(self.internal.get_y().unwrap() as f64 / 127.0) }
}

impl<'a> From<&'a vex_rt::controller::AnalogStick> for AnalogStick<'a> {
	fn from(value: &'a vex_rt::controller::AnalogStick) -> Self { Self { internal: &value } }
}

pub struct Button<'a> {
	internal: &'a vex_rt::controller::Button,
}

impl<'a> From<&'a vex_rt::controller::Button> for Button<'a> {
	fn from(value: &'a vex_rt::controller::Button) -> Self { Self { internal: &value } }
}

impl<'a> Button<'a> {
	pub fn is_pressed(&self) -> bool { self.internal.is_pressed().unwrap() }
}

pub struct Controller {
	internal: vex_rt::controller::Controller,
}

impl From<vex_rt::controller::Controller> for Controller {
	fn from(controller: vex_rt::controller::Controller) -> Self { Self { internal: controller } }
}

macro_rules! controller_impl {
	($type:ident, $($name:ident),+) => {
		impl Controller {
			$(pub fn $name(&self) -> $type {
				$type {
					internal: &self.internal.$name
				}
			})+
		}
	};
}

controller_impl!(AnalogStick, left_stick, right_stick);
controller_impl!(Button, l1, l2, r1, r2, up, down, left, right, x, y, a, b);
