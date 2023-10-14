use core::fmt::Display;

use uom::si::{
	angle::{degree, radian},
	f64::{Angle, Length},
	length::inch,
};

use crate::math::{Real, RealSquare};

#[derive(Copy, Clone, Default, Debug)]
pub struct Position(pub Coordinates, pub Angle);

impl Position {
	pub fn new(x: Length, y: Length, heading: Angle) -> Self { Self(Coordinates { x, y }, heading) }
}

#[derive(Copy, Clone, Default, Debug)]
pub struct Coordinates {
	pub x: Length,
	pub y: Length,
}

impl Coordinates {
	pub fn new(x: Length, y: Length) -> Self { Self { x, y } }

	pub fn distance_to(&self, other: &Self) -> Length {
		let dx: Length = self.x - other.x;
		let dy: Length = self.y - other.y;

		(dx * dx + dy * dy).sqrt()
	}

	pub fn angle_to(&self, other: &Self) -> Angle {
		let dx: Length = other.x - self.x;
		let dy: Length = other.y - self.y;

		return Angle::HALF_TURN / 2.0 - dy.atan2(dx);
	}
}
