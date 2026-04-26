//! Unit-typed numeric newtypes.
//!
//! All are `#[repr(transparent)]` over their inner primitive, so a
//! `Vec<DegPerSec>` has the same layout as `Vec<f64>`. Use
//! [`bytemuck::cast_slice`] to peel the newtype off in zero cost when
//! handing data to numeric kernels (FFT, ndarray, etc.).
//!
//! ```rust,ignore
//! let raw: &[f64] = bytemuck::cast_slice(&gyro_series.values);
//! ```

use std::iter::Sum;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

/// Defines a `#[repr(transparent)]` newtype over a primitive number with
/// the standard arithmetic impls (Add, Sub, Mul/Div by inner scalar, Neg, Sum).
macro_rules! unit_newtype_f {
    ($name:ident, $inner:ty, $doc:literal) => {
        #[doc = $doc]
        #[repr(transparent)]
        #[derive(
            Clone,
            Copy,
            Debug,
            Default,
            PartialEq,
            PartialOrd,
            Pod,
            Zeroable,
            Serialize,
            Deserialize,
        )]
        pub struct $name(pub $inner);

        impl $name {
            #[inline]
            pub const fn new(v: $inner) -> Self {
                Self(v)
            }
            #[inline]
            pub const fn get(self) -> $inner {
                self.0
            }
        }

        impl From<$inner> for $name {
            #[inline]
            fn from(v: $inner) -> Self {
                Self(v)
            }
        }

        impl From<$name> for $inner {
            #[inline]
            fn from(v: $name) -> Self {
                v.0
            }
        }

        impl Add for $name {
            type Output = Self;
            #[inline]
            fn add(self, o: Self) -> Self {
                Self(self.0 + o.0)
            }
        }

        impl AddAssign for $name {
            #[inline]
            fn add_assign(&mut self, o: Self) {
                self.0 += o.0;
            }
        }

        impl Sub for $name {
            type Output = Self;
            #[inline]
            fn sub(self, o: Self) -> Self {
                Self(self.0 - o.0)
            }
        }

        impl SubAssign for $name {
            #[inline]
            fn sub_assign(&mut self, o: Self) {
                self.0 -= o.0;
            }
        }

        impl Mul<$inner> for $name {
            type Output = Self;
            #[inline]
            fn mul(self, k: $inner) -> Self {
                Self(self.0 * k)
            }
        }

        impl Div<$inner> for $name {
            type Output = Self;
            #[inline]
            fn div(self, k: $inner) -> Self {
                Self(self.0 / k)
            }
        }

        impl Neg for $name {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }

        impl Sum for $name {
            #[inline]
            fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
                Self(iter.map(|v| v.0).sum())
            }
        }
    };
}

/// Same as [`unit_newtype_f`] but for unsigned integer wrappers (no Neg, no Div).
macro_rules! unit_newtype_u {
    ($name:ident, $inner:ty, $doc:literal) => {
        #[doc = $doc]
        #[repr(transparent)]
        #[derive(
            Clone,
            Copy,
            Debug,
            Default,
            Eq,
            PartialEq,
            Ord,
            PartialOrd,
            Hash,
            Pod,
            Zeroable,
            Serialize,
            Deserialize,
        )]
        pub struct $name(pub $inner);

        impl $name {
            #[inline]
            pub const fn new(v: $inner) -> Self {
                Self(v)
            }
            #[inline]
            pub const fn get(self) -> $inner {
                self.0
            }
        }

        impl From<$inner> for $name {
            #[inline]
            fn from(v: $inner) -> Self {
                Self(v)
            }
        }

        impl From<$name> for $inner {
            #[inline]
            fn from(v: $name) -> Self {
                v.0
            }
        }
    };
}

unit_newtype_u!(Microseconds, u64, "Time, microseconds since session start.");
unit_newtype_f!(DegPerSec, f64, "Angular velocity, degrees per second.");
unit_newtype_f!(
    Radians,
    f64,
    "Angle in radians (used at parse boundaries before normalising to deg/s)."
);
unit_newtype_f!(MetersPerSec2, f64, "Linear acceleration, m/s².");
unit_newtype_f!(Volts, f32, "Voltage, volts.");
unit_newtype_f!(Amps, f32, "Current, amperes.");
unit_newtype_f!(Celsius, f32, "Temperature, °C.");
unit_newtype_f!(
    DecimalDegrees,
    f64,
    "Geographic latitude/longitude in decimal degrees."
);
unit_newtype_f!(Meters, f32, "Distance/altitude in metres.");
unit_newtype_f!(MetersPerSec, f32, "Linear speed, m/s.");
unit_newtype_f!(
    Normalized01,
    f32,
    "Unit-normalised value in [0, 1] (motor command, stick deflection)."
);
unit_newtype_u!(
    PwmMicros,
    u16,
    "PWM pulse width, microseconds (typically 1000–2000)."
);
unit_newtype_u!(
    Erpm,
    u32,
    "Electrical RPM (multiply by motor pole pairs to get mechanical RPM)."
);
unit_newtype_u!(Rpm, u32, "Mechanical revolutions per minute.");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cast_slice_is_zero_cost() {
        let v: Vec<DegPerSec> = vec![DegPerSec(1.0), DegPerSec(2.0), DegPerSec(3.0)];
        let raw: &[f64] = bytemuck::cast_slice(&v);
        assert_eq!(raw, &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn arithmetic_compiles() {
        let a = DegPerSec(10.0);
        let b = DegPerSec(3.0);
        assert_eq!((a - b).0, 7.0);
        assert_eq!((a + b).0, 13.0);
        assert_eq!((a * 2.0).0, 20.0);
        assert_eq!((a / 2.0).0, 5.0);
        assert_eq!((-a).0, -10.0);
        let total: DegPerSec = vec![a, b].into_iter().sum();
        assert_eq!(total.0, 13.0);
    }

    #[test]
    fn radians_to_degrees_round_trip() {
        let r = Radians(std::f64::consts::PI);
        let deg = DegPerSec(r.0.to_degrees());
        assert!((deg.0 - 180.0).abs() < 1e-12);
    }
}
