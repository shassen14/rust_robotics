/// Converts an assumed angle's units from radians to degrees
///
/// # Arguments
///
/// * `angle` - Angle in radians
/// * Returns Angle in degrees
///
pub fn rad_to_deg<T>(angle: T) -> T
where
    f64: std::ops::Mul<T, Output = T>,
{
    (180f64 / std::f64::consts::PI) * angle
}

/// Converts an assumed angle's units from degrees to radians
///
/// # Arguments
///
/// * `angle` - Angle in degrees
/// * Returns Angle in radians
///
pub fn deg_to_rad<T>(angle: T) -> T
where
    f64: std::ops::Mul<T, Output = T>,
{
    (std::f64::consts::PI / 180f64) * angle
}
