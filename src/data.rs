use alloc::format;
use alloc::string::{String, ToString};

use core::ptr::from_ref;
use embedded_sdmmc::{BlockDevice, SdCardError, TimeSource, VolumeManager};

use nalgebra::{Quaternion, UnitQuaternion};

use thiserror::Error;

use uom::ConstZero;
use uom::si::angle::radian;
use uom::si::f32::{Angle as Anglef32, Length as Lengthf32};
use uom::si::f64::{Angle as Anglef64, Length as Lengthf64};
use uom::si::length::meter;

use world_magnetic_model::GeomagneticField;
use world_magnetic_model::time::Date;
use world_magnetic_model::time::error::ComponentRange;

use crate::locations::ruler::{NORTH_POLE_ECEF, ecef_polar_diff, lat_long_to_ecef};
use crate::locations::{LocNode, LocParseError, LocTree};

#[derive(Debug, Clone)]
pub struct Data<'a> {
    pub name_cache_tag: Option<&'a LocNode>,
    pub name: String,
    pub dist: String,
    pub true_heading_rad: f32,
    pub dist_m: u32,
    pub bearing_rad: f32,
    pub bat: u8,
}

#[derive(Debug, Error)]
pub enum DataError {
    #[error("GPS date was invalid: {0}")]
    InvalidDate(ComponentRange),
    #[error("error finding magnetic declination: {0}")]
    CompassError(world_magnetic_model::Error),
    #[error("could not load nearest location: {0}")]
    LocParseError(LocParseError),
}

impl From<ComponentRange> for DataError {
    fn from(value: ComponentRange) -> Self {
        Self::InvalidDate(value)
    }
}

impl From<world_magnetic_model::Error> for DataError {
    fn from(value: world_magnetic_model::Error) -> Self {
        Self::CompassError(value)
    }
}

impl From<LocParseError> for DataError {
    fn from(value: LocParseError) -> Self {
        Self::LocParseError(value)
    }
}

pub fn parse_and_calc<'a, D: BlockDevice<Error = SdCardError>, T: TimeSource>(
    // yaw: Anglef32,
    orientation_quat: [f32; 4],
    gps_loc: [Anglef64; 2],
    gps_hgt: Lengthf32,
    gps_date: Date,
    bat_raw: (u16, (u16, u16)),
    tree: Option<(&VolumeManager<D, T>, &'a LocTree)>,
    cached_string: Option<(&'a LocNode, String)>,
) -> Result<Data<'a>, DataError> {
    let orientation = UnitQuaternion::new_unchecked(Quaternion::new(
        orientation_quat[3],
        orientation_quat[0],
        orientation_quat[1],
        orientation_quat[2],
    ));
    let orientation_euler = orientation.euler_angles();
    let yaw = Anglef32::new::<radian>(orientation_euler.2);

    // Ignore failures for when model becomes outdated
    let declination = match GeomagneticField::new(
        gps_hgt,
        Anglef32::new::<radian>(gps_loc[0].value as f32),
        Anglef32::new::<radian>(gps_loc[1].value as f32),
        gps_date,
    ) {
        Ok(f) => f.declination(),
        Err(e) => {
            defmt::error!("Failed to get declination: {}", e.to_string().as_str());
            Anglef32::ZERO
        }
    };
    let true_hdg: Anglef32 = declination - yaw;

    let gps_loc_ecef = lat_long_to_ecef(&gps_loc);

    let (nct, name, dist, dist_m, abs_bearing_rad) = if let Some(tree) = tree {
        if let Some(node) = tree.1.nearest(&gps_loc_ecef) {
            let l = [
                Lengthf64::new::<meter>(node.geom()[0]),
                Lengthf64::new::<meter>(node.geom()[1]),
                Lengthf64::new::<meter>(node.geom()[2]),
            ];

            let (nct, name) = if let Some((n, s)) = cached_string
                && from_ref(n) == from_ref(node)
            {
                (Some(node), s)
            } else {
                (Some(node), tree.1.name(tree.0, node)?)
            };

            let (dist_m, bearing_rad) = ecef_polar_diff(&gps_loc_ecef, &l);
            let dist_dm = (dist_m.value * 10.0) as u32;

            let dist = if dist_dm >= 15000 {
                format!("{}.{} km", dist_dm / 10000, (dist_dm % 10000) / 1000)
            } else {
                format!("{}.{} m", dist_dm / 10, dist_dm % 10)
            };

            (nct, name, dist, dist_dm / 10, bearing_rad)
        } else {
            let (dist_m, bearing_rad) = ecef_polar_diff(&gps_loc_ecef, &NORTH_POLE_ECEF);
            (
                None,
                "No Locs".to_string(),
                format!("{:.6} {:.6}", gps_loc[0].value, gps_loc[1].value),
                dist_m.value as u32,
                bearing_rad,
            )
        }
    } else {
        let (dist_m, bearing_rad) = ecef_polar_diff(&gps_loc_ecef, &NORTH_POLE_ECEF);
        (
            None,
            "No SD".to_string(),
            format!("{:.6} {:.6}", gps_loc[0].value, gps_loc[1].value),
            dist_m.value as u32,
            bearing_rad,
        )
    };

    let bearing_rad = abs_bearing_rad.value as f32 + yaw.value;

    Ok(Data {
        name_cache_tag: nct,
        name,
        dist,
        true_heading_rad: true_hdg.value,
        bat: bat_level_25(bat_raw.0, bat_raw.1),
        dist_m,
        bearing_rad,
    })
}

// Charge proportion (0 to 1) given voltage approximately follows
//      c(v) = 3v - 11 if v < 3.85 else 1.3v - 4.45
// From https://static.rcgroups.net/forums/attachments/6/4/2/6/9/4/a9404658-9-lipo_capacity.png
// Given raw counts m, and range (r_1, r_2) representing the counts at 0V and 6.6V respectively
// (before the voltage divider), v = 6.6 * (m - r_1) / (r_2 - r_1)
// We want scaled proportion s, here c * 25
//
// Now to reorder to minimize integer scaling error
// Let r = r_2 - r_1, m_s = m - r_1
// switchpoint: m_s = r * (3.85 / 6.6) = (r * 7) / 12
// scalers: 3 * 6.6 * 25 = 495, and 1.3 * 6.6 * 25 = 214.5 = 429 / 2
// offsets: 11 * 25 = 275 and 4.45 * 25 = 111.25 ~= 111
// Therefore s_1 = ((495 * m_s) / r) - 275 and s_2 = ((429 * m_s) / (2 * r)) - 111
fn bat_level_25(raw_val: u16, range: (u16, u16)) -> u8 {
    // Upgrade to u32 for headroom
    let range_abs = (range.1 - range.0) as u32;
    let measure_shifted = raw_val.saturating_sub(range.0) as u32;
    let measure_shiftpoint = (range_abs * 7) / 12;
    if measure_shifted <= measure_shiftpoint {
        ((495 * measure_shifted) / range_abs).saturating_sub(275) as u8
    } else {
        (((429 * measure_shifted) / (2 * range_abs)) - 111) as u8
    }
}
