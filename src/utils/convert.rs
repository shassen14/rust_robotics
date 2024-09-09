/// Converts an assumed angle's units from radians to degrees
///
/// # Arguments
///
/// * `angle` - Angle in radians
/// * Returns Angle in degrees
///
pub fn rad_to_deg<T>(angle: T) -> T
where
    T: num_traits::Float,
{
    (T::from(180).unwrap() / T::from(std::f64::consts::PI).unwrap()) * angle
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
    T: num_traits::Float,
{
    (T::from(std::f64::consts::PI).unwrap() / T::from(180).unwrap()) * angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_deg_to_rad_f64() {
        let angle_deg = [0., 45., 90., 135., 180.];
        let angle_rad = [
            0.,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_2,
            3. * std::f64::consts::FRAC_PI_4,
            std::f64::consts::PI,
        ];

        for i in 0..5 {
            assert_relative_eq!(deg_to_rad(angle_deg[i]), angle_rad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_rad_to_deg_f64() {
        let angle_deg = [0., 45., 90., 135., 180.];
        let angle_rad = [
            0.,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_2,
            3. * std::f64::consts::FRAC_PI_4,
            std::f64::consts::PI,
        ];

        for i in 0..5 {
            assert_relative_eq!(rad_to_deg(angle_rad[i]), angle_deg[i], epsilon = 1e-12);
        }
    }
}
