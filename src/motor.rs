use core::fmt::Debug;

use uom::si::{
	angle::degree,
	angular_velocity::revolution_per_minute,
	electric_current::milliampere,
	electric_potential::{millivolt, volt},
	f64::{Angle, AngularVelocity, ElectricCurrent, ElectricPotential, Power, Ratio, ThermodynamicTemperature, Torque},
	power::watt,
	ratio::{percent, ratio},
	thermodynamic_temperature::degree_celsius,
	torque::newton_meter,
};
use vex_rt::{motor::Gearset, prelude::BrakeMode, smart_port::SmartPort};

pub struct Motor {
	internal: vex_rt::motor::Motor,
}

impl Debug for Motor {
	fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result { f.debug_struct("Motor").finish() }
}

impl Motor {
	pub fn new(port: SmartPort, gearset: Gearset, reversed: bool) -> Self {
		Self {
			internal: port.into_motor(gearset, vex_rt::prelude::EncoderUnits::Degrees, reversed),
		}
	}

	pub fn move_ratio(&mut self, value: Ratio) {
		self.internal
			.move_voltage((12000.0 * value.get::<ratio>()) as i32)
			.unwrap();
	}

	pub fn move_voltage(&mut self, voltage: ElectricPotential) {
		self.internal.move_voltage(voltage.get::<millivolt>() as i32).unwrap();
	}

	pub fn move_velocity(&mut self, velocity: AngularVelocity) {
		self.internal
			.move_velocity(velocity.get::<revolution_per_minute>() as i32)
			.unwrap();
	}

	pub fn move_relative(&mut self, position: Angle, velocity: AngularVelocity) {
		self.internal
			.move_relative(position.get::<degree>(), velocity.get::<revolution_per_minute>() as i32)
			.unwrap();
	}

	pub fn move_absolute(&mut self, position: Angle, velocity: AngularVelocity) {
		self.internal
			.move_absolute(position.get::<degree>(), velocity.get::<revolution_per_minute>() as i32)
			.unwrap();
	}

	pub fn modify_profiled_velocity(&mut self, velocity: AngularVelocity) {
		self.internal
			.modify_profiled_velocity(velocity.get::<revolution_per_minute>() as i32)
			.unwrap();
	}

	pub fn get_actual_velocity(&self) -> AngularVelocity {
		self.internal
			.get_actual_velocity()
			.map_or(Default::default(), |velocity| {
				AngularVelocity::new::<revolution_per_minute>(velocity as f64)
			})
	}

	pub fn get_current_draw(&self) -> ElectricCurrent {
		self.internal
			.get_actual_velocity()
			.map_or(Default::default(), |current| {
				ElectricCurrent::new::<milliampere>(current)
			})
	}

	pub fn get_efficiency(&self) -> Ratio {
		self.internal
			.get_efficiency()
			.map_or(Default::default(), |efficiency| Ratio::new::<percent>(efficiency))
	}

	pub fn get_position(&self) -> Angle {
		self.internal
			.get_position()
			.map_or(Default::default(), |position| Angle::new::<degree>(position))
	}

	pub fn get_power(&self) -> Power {
		self.internal
			.get_power()
			.map_or(Default::default(), |power| Power::new::<watt>(power))
	}

	pub fn get_temperature(&self) -> ThermodynamicTemperature {
		self.internal
			.get_temperature()
			.map_or(Default::default(), |temperature| {
				ThermodynamicTemperature::new::<degree_celsius>(temperature)
			})
	}

	pub fn get_torque(&self) -> Torque {
		self.internal
			.get_torque()
			.map_or(Default::default(), |torque| Torque::new::<newton_meter>(torque))
	}

	pub fn get_voltage(&self) -> ElectricPotential {
		self.internal.get_voltage().map_or(Default::default(), |voltage| {
			ElectricPotential::new::<millivolt>(voltage as f64)
		})
	}

	pub fn is_over_temp(&self) -> bool { self.internal.is_over_temp().unwrap_or_default() }

	pub fn get_current_limit(&self) -> ElectricCurrent {
		self.internal
			.get_current_limit()
			.map_or(ElectricCurrent::new::<milliampere>(2500.0), |current| {
				ElectricCurrent::new::<milliampere>(current as f64)
			})
	}

	pub fn get_voltage_limit(&self) -> Option<ElectricPotential> {
		let voltage = self.internal.get_voltage_limit().unwrap_or_default();
		if voltage == 0 {
			None
		} else {
			Some(ElectricPotential::new::<volt>(voltage as f64))
		}
	}

	pub fn set_brake_mode(&mut self, mode: BrakeMode) { self.internal.set_brake_mode(mode).unwrap(); }

	pub fn set_current_limit(&mut self, limit: ElectricCurrent) {
		self.internal
			.set_current_limit(limit.get::<milliampere>() as i32)
			.unwrap();
	}

	pub fn set_voltage_limit(&mut self, limit: ElectricPotential) {
		self.internal.set_voltage_limit(limit.get::<volt>() as i32).unwrap();
	}

	pub fn set_zero_position(&mut self, position: Angle) {
		self.internal.set_zero_position(position.get::<degree>()).unwrap();
	}

	pub fn tare_position(&mut self) { self.internal.tare_position().unwrap(); }
}
