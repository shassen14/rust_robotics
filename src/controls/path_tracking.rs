use nalgebra::{self as na};
use serde::{Deserialize, Serialize};

// TODO: place pub struct somewhere else to save paths

#[derive(Debug, Serialize, Deserialize)]
pub struct RecordPoint<T> {
    pub time: T,
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> RecordPoint<T> {
    pub fn new(time: T, x: T, y: T, z: T) -> Self {
        RecordPoint {
            time: time,
            x: x,
            y: y,
            z: z,
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// TODO: Make this a class? Not sure if I can generalize this for other
// path tracking classes
///////////////////////////////////////////////////////////////////////////////

pub fn calculate_lookahead_point<T>(
    path: &Vec<na::Point3<T>>,
    position_current: &na::Point3<T>,
    start_index: usize,
    lookahead_distance: T,
) -> (usize, T, na::Point3<T>)
where
    T: na::RealField + std::fmt::Debug + Copy + 'static,
{
    let lookahead_distance_squared: T = lookahead_distance * lookahead_distance;
    for i in start_index..path.len() {
        let ds = na::distance_squared(position_current, &path[i]);
        if ds >= lookahead_distance_squared {
            return (i, na::distance(position_current, &path[i]), path[i]);
        }
    }

    // return the last point in path if no other point meets the required lookahead distance
    return (
        path.len() - 1,
        na::distance(position_current, &path[path.len() - 1]),
        path[path.len() - 1],
    );
}

pub fn pure_pursuit<T>(
    position_current: &na::Point2<T>,
    position_target: &na::Point2<T>,
    yaw_current: T,
    target_distance: T,
    wheelbase: T,
) -> T
where
    T: na::RealField + std::fmt::Debug + Copy + 'static,
{
    let yaw_desired = calculate_desired_yaw(position_current, position_target);
    let yaw_relative = yaw_desired - yaw_current;
    // let curvature = (T::simd_sin(yaw_relative) + T::simd_sin(yaw_relative)) / target_distance;
    let curvature = (T::sin(yaw_relative) + T::sin(yaw_relative)) / target_distance;
    let rwa = T::atan2(wheelbase * curvature, wheelbase / wheelbase);
    rwa
}

fn calculate_desired_yaw<T>(position_current: &na::Point2<T>, position_target: &na::Point2<T>) -> T
where
    T: na::RealField + Clone + Copy,
{
    T::atan2(
        position_target.y - position_current.y,
        position_target.x - position_current.x,
    )
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::convert;
    use approx::assert_relative_eq;

    #[test]
    fn path_tracking_lookahead_point() {
        let path = vec![
            na::Point3::new(0., 0., 0.),
            na::Point3::new(3., 3., 0.),
            na::Point3::new(6., 6., 0.),
            na::Point3::new(9., 9., 0.),
            na::Point3::new(12., 12., 0.),
            na::Point3::new(15., 15., 0.),
        ];

        // first answer from the beginning
        let position1 = na::Point3::new(0., 0., 0.);
        let start_index1 = 0usize;
        let lookahead_distance: f64 = 5f64;

        let estimation1 =
            calculate_lookahead_point::<f64>(&path, &position1, start_index1, lookahead_distance);

        let answer1 = (2usize, f64::sqrt(72.), path[2usize]);

        assert_eq!(estimation1.0, answer1.0);
        assert_relative_eq!(estimation1.1, answer1.1, epsilon = 1e-12);
        for i in 0..2 {
            assert_relative_eq!(estimation1.2[i], answer1.2[i], epsilon = 1e-12);
        }

        // second answer from another starting index and position
        let position2 = na::Point3::new(13., 13., 0.);
        let start_index2 = 4usize;

        let estimation2 =
            calculate_lookahead_point::<f64>(&path, &position2, start_index2, lookahead_distance);

        // sqrt (2^2 + 2^2) because of 15 - 13
        let answer2 = (5usize, f64::sqrt(8.), path[5usize]);

        assert_eq!(estimation2.0, answer2.0);
        assert_relative_eq!(estimation2.1, answer2.1, epsilon = 1e-12);
        for i in 0..2 {
            assert_relative_eq!(estimation2.2[i], answer2.2[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn path_tracking_desired_yaw() {
        let position1 = na::Point2::new(0., 0.);
        let target1 = na::Point2::new(1., 1.);

        let desired_yaw1 = calculate_desired_yaw(&position1, &target1);
        let answer1 = convert::deg_to_rad(45.);

        assert_relative_eq!(desired_yaw1, answer1, epsilon = 1e-12);

        let position2 = na::Point2::new(0., 0.);
        let target2 = na::Point2::new(-1., -1.);

        let desired_yaw2 = calculate_desired_yaw(&position2, &target2);
        let answer2 = convert::deg_to_rad(-135.);

        assert_relative_eq!(desired_yaw2, answer2, epsilon = 1e-12);
    }

    #[test]
    fn path_tracking_pure_pursuit() {
        let position_current = na::Point2::new(0., 0.);
        let position_target = na::Point2::new(1., 0.);

        let yaw_current: f64 = 0.;
        let target_distance: f64 = 5.0; // m
        let wheelbase = 2.5; // m

        let rwa1 = pure_pursuit(
            &position_current,
            &position_target,
            yaw_current,
            target_distance,
            wheelbase,
        );
        let answer1 = 0.0;

        assert_relative_eq!(rwa1, answer1, epsilon = 1e-12);
    }
}
