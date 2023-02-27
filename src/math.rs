use core::marker::PhantomData;

use libm::{acos, asin, atan, atan2, cos, sin, sqrt, tan};
use uom::{
	num_traits::Num,
	si::{
		amount_of_substance::mole,
		angle::radian,
		electric_current::ampere,
		length::meter,
		luminous_intensity::candela,
		mass::kilogram,
		quantities::{Angle, Ratio},
		ratio::ratio,
		thermodynamic_temperature::kelvin,
		time::second,
		Dimension,
		Quantity,
		ISQ,
		SI,
	},
	typenum::{Integer, PartialDiv, PartialQuot, P2},
	Conversion,
};

pub trait Real {
	type Angle: RealAngle;

	fn atan2(self, x: Self) -> Self::Angle;
}

pub trait RealSquare: Real {
	type Root;

	fn sqrt(self) -> Self::Root;
}

pub trait RealAngle {
	type Ratio: RealRatio;

	fn sin(self) -> Self::Ratio;
	fn cos(self) -> Self::Ratio;
	fn tan(self) -> Self::Ratio;
}

pub trait RealRatio: RealSquare<Root = Self> {
	fn asin(self) -> Self::Angle;
	fn acos(self) -> Self::Angle;
	fn atan(self) -> Self::Angle;
}

pub trait RealPrimitive: RealAngle<Ratio = Self> + RealRatio<Angle = Self> {}

impl Real for f64 {
	type Angle = Self;

	fn atan2(self, x: Self) -> Self::Angle { atan2(self, x) }
}

impl RealSquare for f64 {
	type Root = Self;

	fn sqrt(self) -> Self::Root {
		// unsafe { sqrtf64(self) }
		sqrt(self)
	}
}

impl RealAngle for f64 {
	type Ratio = Self;

	fn sin(self) -> Self::Ratio { sin(self) }

	fn cos(self) -> Self::Ratio { cos(self) }

	fn tan(self) -> Self::Ratio { tan(self) }
}

impl RealRatio for f64 {
	fn asin(self) -> Self::Angle { asin(self) }

	fn acos(self) -> Self::Angle { acos(self) }

	fn atan(self) -> Self::Angle { atan(self) }
}

impl RealPrimitive for f64 {}

impl<V: RealPrimitive + Conversion<V> + Num, D: Dimension + ?Sized> Real for Quantity<D, SI<V>, V>
where
	radian: Conversion<V, T = V::T>,
	ratio: Conversion<V, T = V::T>,
	meter: Conversion<V, T = V::T>,
	kilogram: Conversion<V, T = V::T>,
	second: Conversion<V, T = V::T>,
	ampere: Conversion<V, T = V::T>,
	kelvin: Conversion<V, T = V::T>,
	mole: Conversion<V, T = V::T>,
	candela: Conversion<V, T = V::T>,
{
	type Angle = Angle<V>;

	fn atan2(self, x: Self) -> Self::Angle {
		Self::Angle {
			dimension: PhantomData,
			units: PhantomData,
			value: self.value.atan2(x.value),
		}
	}
}

impl<V: RealPrimitive + Conversion<V> + Num, D: Dimension + ?Sized> RealSquare for Quantity<D, SI<V>, V>
where
	radian: Conversion<V, T = V::T>,
	ratio: Conversion<V, T = V::T>,
	meter: Conversion<V, T = V::T>,
	kilogram: Conversion<V, T = V::T>,
	second: Conversion<V, T = V::T>,
	ampere: Conversion<V, T = V::T>,
	kelvin: Conversion<V, T = V::T>,
	mole: Conversion<V, T = V::T>,
	candela: Conversion<V, T = V::T>,
	D::L: PartialDiv<P2>,
	<D::L as PartialDiv<P2>>::Output: Integer,
	D::M: PartialDiv<P2>,
	<D::M as PartialDiv<P2>>::Output: Integer,
	D::T: PartialDiv<P2>,
	<D::T as PartialDiv<P2>>::Output: Integer,
	D::I: PartialDiv<P2>,
	<D::I as PartialDiv<P2>>::Output: Integer,
	D::Th: PartialDiv<P2>,
	<D::Th as PartialDiv<P2>>::Output: Integer,
	D::N: PartialDiv<P2>,
	<D::N as PartialDiv<P2>>::Output: Integer,
	D::J: PartialDiv<P2>,
	<D::J as PartialDiv<P2>>::Output: Integer,
{
	type Root = Quantity<
		ISQ<
			PartialQuot<D::L, P2>,
			PartialQuot<D::M, P2>,
			PartialQuot<D::T, P2>,
			PartialQuot<D::I, P2>,
			PartialQuot<D::Th, P2>,
			PartialQuot<D::N, P2>,
			PartialQuot<D::J, P2>,
		>,
		SI<V>,
		V,
	>;

	fn sqrt(self) -> Self::Root {
		Self::Root {
			dimension: PhantomData,
			units: PhantomData,
			value: self.value.sqrt(),
		}
	}
}

impl<V: RealPrimitive + RealRatio + Conversion<V> + Num> RealAngle for Angle<V>
where
	radian: Conversion<V, T = V::T>,
	ratio: Conversion<V, T = V::T>,
	meter: Conversion<V, T = V::T>,
	kilogram: Conversion<V, T = V::T>,
	second: Conversion<V, T = V::T>,
	ampere: Conversion<V, T = V::T>,
	kelvin: Conversion<V, T = V::T>,
	mole: Conversion<V, T = V::T>,
	candela: Conversion<V, T = V::T>,
{
	type Ratio = Ratio<V>;

	fn sin(self) -> Self::Ratio { Self::Ratio::new::<ratio>(self.get::<radian>().sin()) }

	fn cos(self) -> Self::Ratio { Self::Ratio::new::<ratio>(self.get::<radian>().cos()) }

	fn tan(self) -> Self::Ratio { Self::Ratio::new::<ratio>(self.get::<radian>().tan()) }
}

impl<V: RealPrimitive + Conversion<V> + Num> RealRatio for Ratio<V>
where
	radian: Conversion<V, T = V::T>,
	ratio: Conversion<V, T = V::T>,
	meter: Conversion<V, T = V::T>,
	kilogram: Conversion<V, T = V::T>,
	second: Conversion<V, T = V::T>,
	ampere: Conversion<V, T = V::T>,
	kelvin: Conversion<V, T = V::T>,
	mole: Conversion<V, T = V::T>,
	candela: Conversion<V, T = V::T>,
{
	fn asin(self) -> Self::Angle { Self::Angle::new::<radian>(self.get::<ratio>().asin()) }

	fn acos(self) -> Self::Angle { Self::Angle::new::<radian>(self.get::<ratio>().acos()) }

	fn atan(self) -> Self::Angle { Self::Angle::new::<radian>(self.get::<ratio>().atan()) }
}
