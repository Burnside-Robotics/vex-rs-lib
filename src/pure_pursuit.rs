use alloc::vec::Vec;

use uom::{
	si::{
		angle::degree,
		f64::{Angle, Area, Length},
		length::inch,
	},
	ConstZero,
};
use vex_rt::prelude::println;

use crate::{
	coordinates::{Coordinates, Position},
	math::RealSquare,
};

#[derive(Clone, Copy)]
pub struct PurePursuitCommands(pub Length, pub Angle);

pub struct PurePursuitSystem {
	current_node: i32,
	sequence: Option<Vec<Coordinates>>,

	look_ahead_distance: Length,

	pub enabled: bool,
}

impl PurePursuitSystem {
	pub fn new() -> Self {
		Self {
			current_node: 0,
			sequence: None,
			look_ahead_distance: Length::new::<inch>(20.0),
			enabled: false,
		}
	}

	pub fn set_sequence(&mut self, sequence: Option<Vec<Coordinates>>) {
		self.sequence = sequence;
		self.current_node = 0;
	}

	fn get_target_point(
		&self, previous_point: Coordinates, next_point: Coordinates, robot_position: Coordinates,
	) -> Coordinates {
		let x1: Length = previous_point.x - robot_position.x;
		let y1: Length = previous_point.y - robot_position.y;
		let x2: Length = next_point.x - robot_position.x;
		let y2: Length = next_point.y - robot_position.y;

		let dx: Length = x2 - x1;
		let dy: Length = y2 - y1;

		let dr: Length = (dx * dx + dy * dy).sqrt();

		let d: Area = x1 * y2 - x2 * y1;

		let discriminant = self.look_ahead_distance * self.look_ahead_distance * dr * dr - d * d;

		let signy = if dy >= Length::ZERO { 1.0 } else { -1.0 };

		if discriminant.value >= 0.0 {
			let target_point1 = Coordinates::new(
				(d * dy + signy * dx * discriminant.sqrt()) / (dr * dr) + robot_position.x,
				(-d * dx + dy.abs() * discriminant.sqrt()) / (dr * dr) + robot_position.y,
			);

			let target_point2 = Coordinates::new(
				(d * dy - signy * dx * discriminant.sqrt()) / (dr * dr) + robot_position.x,
				(-d * dx - dy.abs() * discriminant.sqrt()) / (dr * dr) + robot_position.y,
			);

			if target_point1.distance_to(&next_point) < target_point2.distance_to(&next_point) {
				target_point1
			} else {
				target_point2
			}
		} else {
			previous_point
		}
	}

	pub fn cycle(&mut self, robot_position: Position) -> Option<PurePursuitCommands> {
		let Position(robot_coordinates, heading) = robot_position;

		if !self.enabled {
			return None;
		}

		if let Some(sequence) = &self.sequence {
			let previous_point = sequence
				.get((self.current_node - 1) as usize)
				.unwrap_or(&robot_coordinates);

			if let Some(next_point) = sequence.get(self.current_node as usize) {
				let target_point = self.get_target_point(*previous_point, *next_point, robot_coordinates);

				let mut turn_error: Angle = robot_coordinates.angle_to(&target_point) - heading;

				if turn_error > Angle::HALF_TURN {
					turn_error -= Angle::FULL_TURN
				} else if turn_error < -Angle::HALF_TURN {
					turn_error += Angle::FULL_TURN
				}

				let distance_error: Length = robot_coordinates.distance_to(&target_point);

				if next_point.distance_to(&robot_coordinates) < self.look_ahead_distance {
					self.current_node += 1;
				}

				println!(
					"Robot: ({:.2}in, {:.2}in, {:.2}deg) Next: ({:.2}in, {:.2}in) Target: ({:.2}in, {:.2}in) {:.2}in, \
					 {:.2}deg",
					robot_coordinates.x.get::<inch>(),
					robot_coordinates.y.get::<inch>(),
					heading.get::<degree>(),
					next_point.x.get::<inch>(),
					next_point.y.get::<inch>(),
					target_point.x.get::<inch>(),
					target_point.y.get::<inch>(),
					distance_error.get::<inch>(),
					turn_error.get::<degree>(),
				);

				return Some(PurePursuitCommands(distance_error, turn_error));
			} else {
				self.current_node = 0;
				self.sequence = None;
			}
		}
		Some(PurePursuitCommands(Length::ZERO, Angle::ZERO))
	}
}
