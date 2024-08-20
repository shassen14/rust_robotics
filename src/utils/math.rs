use crate::utils::defs::AngleUnits;
use crate::utils::transforms::FrameTransform3;
use nalgebra as na;

// not really sure what to call this or where to put these
// functions. Just needing them

/// Bounds a value of the same type between lower and upper bound
///
/// * Arguments
///
/// * `value` - Value to possibly bound
/// * `lower_bound` - Value cannot be less than lower_bound
/// * `upper_bound` - Value cannot be greater than upper_bound
/// * Returns the value between two bounds
///
pub fn bound_value<T>(value: T, lower_bound: T, upper_bound: T) -> T
where
    T: std::fmt::Debug + std::cmp::PartialOrd,
{
    // Error occurs if developer switches upper and lower bound in input
    // TODO: make this more elegant
    assert!(
        lower_bound <= upper_bound,
        "lower bound ({:?}) must be lower than or equal upper bound ({:?})",
        lower_bound,
        upper_bound
    );

    if value < lower_bound {
        return lower_bound;
    }
    if value > upper_bound {
        return upper_bound;
    }
    value
}

/// Calculate 4 corner points given, the center points and lengths
///
/// * Arguments
///
/// * `value` - Value to possibly bound
/// * `lower_bound` - Value cannot be less than lower_bound
/// * `upper_bound` - Value cannot be greater than upper_bound
/// * Returns the value between two bounds
///
pub fn calculate_rectangle_points(
    start_point: &(f64, f64),
    length_front: f64,
    length_rear: f64,
    width_left: f64,
    width_right: f64,
    heading_angle: f64,
    angle_units: AngleUnits,
) -> [(f64, f64); 4] {
    let tf = FrameTransform3::new(
        &[start_point.0, start_point.1, 0., 0., 0., heading_angle],
        angle_units,
    );
    let point_top_left = tf.point_b_to_i(&na::Point3::new(length_front, width_left, 0.));
    let point_top_right = tf.point_b_to_i(&na::Point3::new(length_front, -width_right, 0.));
    let point_bot_left = tf.point_b_to_i(&na::Point3::new(-length_rear, width_left, 0.));
    let point_bot_right = tf.point_b_to_i(&na::Point3::new(-length_rear, -width_right, 0.));

    // return points
    [
        (point_top_left.x, point_top_left.y),
        (point_top_right.x, point_top_right.y),
        (point_bot_right.x, point_bot_right.y),
        (point_bot_left.x, point_bot_left.y),
    ]
}

pub fn calculate_line_endpoints(
    start_point: &(f64, f64),
    length: f64,
    heading_angle: f64,
    angle_units: AngleUnits,
) -> [(f64, f64); 2] {
    let tf = FrameTransform3::new(
        &[start_point.0, start_point.1, 0., 0., 0., heading_angle],
        angle_units,
    );
    let end_point = tf.point_b_to_i(&na::Point3::new(length, 0., 0.));

    // return points
    [(start_point.0, start_point.1), (end_point.x, end_point.y)]
}
