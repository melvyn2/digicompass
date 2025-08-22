use core::marker::PhantomData;

use libm::{atan2, sqrt};

use nalgebra::base::Vector3;

use uom::si::angle::radian;
use uom::si::f64::{Angle, Length, Ratio};
use uom::si::length::meter;
use uom::si::ratio::ratio;

// implicitly meters
const WGS84_SMA: Length = Length {
    dimension: PhantomData,
    units: PhantomData,
    value: 6378137.0,
};

// unitless
const FLATTENING: f64 = 1.0 / 298.257223563;
const ECCENTRICITY_2: f64 = FLATTENING * (2.0 - FLATTENING);
const ECCENTRICITY_2_COMPLEMENT: f64 = 1.0 - ECCENTRICITY_2;

// Calculated as [0.0, 0.0, WGS84_SMA.value * ECCENTRICITY_2_COMPLEMENT.sqrt()]
pub const NORTH_POLE_ECEF: [Length; 3] = [
    Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.0,
    },
    Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.0,
    },
    Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 6356752.314245179,
    },
];

/// lat/long radians to 3d pos in meters
pub fn lat_long_to_ecef(pos: &[Angle; 2]) -> [Length; 3] {
    // let lat: Angle = Angle::new::<radian>(pos[0]);
    // let long: Angle = Angle::new::<radian>(pos[1]);
    let lat = pos[0];
    let long = pos[1];

    let lat_sin: Ratio = lat.sin();
    let lat_cos: Ratio = lat.cos();
    let v: Length =
        WGS84_SMA / (Ratio::new::<ratio>(1.0) - (ECCENTRICITY_2 * lat_sin * lat_sin)).sqrt();

    let x: Length = v * lat_cos * long.cos();
    let y: Length = v * lat_cos * long.sin();
    let z: Length = v * ECCENTRICITY_2_COMPLEMENT * lat_sin;
    [x, y, z]
}

/// ECEF meter coordinates to meter distance, radians east of north bearing
pub fn ecef_polar_diff(from: &[Length; 3], to: &[Length; 3]) -> (Length, Angle) {
    let p = Vector3::new(from[0].value, from[1].value, from[2].value);
    let q = Vector3::new(to[0].value, to[1].value, to[2].value);

    // from https://math.stackexchange.com/a/3330195

    // east and north directions at p
    let e_v = Vector3::new(-p.y, p.x, 0.0).normalize();
    let n_v = p.cross(&e_v).normalize();

    // east and north distances to q
    let e = e_v.dot(&q);
    let n = n_v.dot(&q);

    let bearing = atan2(e, n);
    let dist = sqrt((n * n) + (e * e));
    (Length::new::<meter>(dist), Angle::new::<radian>(bearing))
}

// #[derive(Clone, Debug)]
// pub struct Ruler {
//     // Radius of curvature of our parallel (confusingly also called prime vertical)
//     rc_x: Length,
//     // Radius of curvature of our meridian
//     rc_y: Length,
// }
//
// impl Ruler {
//     /// Latitude, in radians
//     pub fn new(lat: Angle) -> Self {
//         let cos_lat: Ratio = lat.cos();
//         let sin_2_lat: Ratio = Ratio::new::<ratio>(1.0) - (cos_lat * cos_lat);
//         let denom_2: Ratio = 1.0 / (Ratio::new::<ratio>(1.0) - (ECCENTRICITY_2 * sin_2_lat));
//         let denom: Ratio = denom_2.sqrt();
//
//         let rc_x: Length = WGS84_SMA * cos_lat * denom;
//         let rc_y: Length = WGS84_SMA * ECCENTRICITY_2_COMPLEMENT * denom * denom_2;
//         Self { rc_x, rc_y }
//     }
//
//     /// Pair of \[`lat`, `long`\] in radians -> north, east distance in meters
//     pub fn cartesian_diff(&self, from: [Angle; 2], to: [Angle; 2]) -> [Length; 2] {
//         let dx: Length = fmod(to[1].value - from[1].value, PI) * self.rc_x;
//         let dy: Length = fmod(to[0].value - from[0].value, PI / 2.0) * self.rc_y;
//         [dx, dy]
//     }
//
//     /// Pair of \[`lat`, `long`\] in radians -> distance in meters, bearing in radians east of north
//     pub fn polar_diff(&self, from: [Angle; 2], to: [Angle; 2]) -> (Length, Angle) {
//         let cd: [Length; 2] = self.cartesian_diff(from, to);
//         (
//             ((cd[0] * cd[0]) + (cd[1] * cd[1])).sqrt(),
//             cd[0].atan2(cd[1]),
//         )
//     }
// }
