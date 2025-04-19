use crate::utils::defs::AngleUnits;
use crate::utils::transforms::FrameTransform3;
use nalgebra as na;
use num_traits::Float;

// not really sure what to call this or where to put these
// functions. Just needing them

/// Bounds a value of the same type between lower and upper bound
/// [lower, upper]
///
/// * Arguments
///
/// * `value` - Value to possibly bound
/// * `lower_bound` - Value cannot be less than lower_bound
/// * `upper_bound` - Value cannot be greater than upper_bound
/// * Returns the value between two bounds
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

/// Converts angle to be wrapped around the bounds [lower, upper)
///
/// * Arguments
///
/// * `angle` - Value to possibly bound
/// * `lower_bound` - Value cannot be less than lower_bound
/// * `upper_bound` - Value cannot be greater than upper_bound
/// * Returns the angle equivalent to the original wrapped around the bounds
pub fn bound_polar_value<T>(angle: T, lower_bound: T, upper_bound: T) -> T
where
    T: std::cmp::PartialOrd
        + std::ops::Sub<Output = T>
        + std::ops::SubAssign
        + std::ops::AddAssign
        + std::fmt::Debug
        + Copy,
{
    // upper bound should be greater than lower bound
    assert!(
        lower_bound < upper_bound,
        "lower bound ({:?}) must be lower than upper bound ({:?})",
        lower_bound,
        upper_bound
    );

    let period: T = upper_bound - lower_bound;

    // add or subtract until the angle_output is within the bounds
    let mut angle_output = angle;
    while angle_output >= upper_bound || angle_output < lower_bound {
        if angle_output >= upper_bound {
            angle_output -= period;
        } else if angle_output < lower_bound {
            angle_output += period;
        }
    }
    angle_output
}

/// Calculate 4 corner points given, the center points and lengths
///
/// * Arguments
///
/// * `start_point` - 2D point (x, y)
/// * `length_front` - Number of units away from starting point forward (magnitude)
/// * `length_rear` - Number of units away from starting point backwards (magnitude)
/// * `width_left` - Number of units away from starting points to the left (magnitude)
/// * `width_left` - Number of units away from starting points to the right (magnitude)
/// * `heading_angle` - Inertial angle going towards the front (direction)
/// * `angle_units` - Radians or Degrees
/// * Returns four 2D points (forward left -> forward right -> bottom right -> bottom left)
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
        Some(angle_units),
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

/// Calculate 2 end points given the starting point, angle, and length
///
/// * Arguments
///
/// * `start_point` - 2D point (x, y)
/// * `length` - Number of units away from starting point (magnitude)
/// * `heading_angle` - Inertial angle going toward the second point (direction)
/// * `angle_units` - Radians or Degrees
/// * Returns two 2D points where starting point is the first index and end point is the second index
pub fn calculate_line_endpoints(
    start_point: &(f64, f64),
    length: f64,
    heading_angle: f64,
    angle_units: AngleUnits,
) -> [(f64, f64); 2] {
    let tf = FrameTransform3::new(
        &[start_point.0, start_point.1, 0., 0., 0., heading_angle],
        Some(angle_units),
    );
    let end_point = tf.point_b_to_i(&na::Point3::new(length, 0., 0.));

    // return points
    [(start_point.0, start_point.1), (end_point.x, end_point.y)]
}

/// Simple numerical gradient
///
/// * Arguments
///
/// * `values` - A vector of values for which the gradient is calculated.
/// * `base` - A vector of values that are used as the base of the gradient calculation.
///
/// * Returns a vector of the same length as `values` and `base` containing the gradient
///   calculated at each point.
///
/// The gradient is calculated using a forward difference at the start, central differences
/// for the middle points, and a backward difference at the end. If the base values are equal
/// to each other for a particular point, the gradient is set to 0.
pub fn gradient_1d<T>(values: &Vec<T>, base: &Vec<T>) -> Vec<T>
where
    T: Float,
{
    if values.len() != base.len() {
        panic!(
            "Input vectors must have the same length, values: {}, base: {}",
            values.len(),
            base.len()
        );
    }

    let n = values.len();
    let m = base.len();
    let mut grad = vec![T::zero(); n];

    if n == 0 || m == 0 || n == 1 || m == 1 {
        return grad;
    }

    // Forward difference at start
    if base[1] != base[0] {
        grad[0] = (values[1] - values[0]) / (base[1] - base[0]);
    }

    // Central differences for middle points
    for i in 1..n - 1 {
        if base[i + 1] != base[i - 1] {
            let ds = base[i + 1] - base[i - 1];
            grad[i] = (values[i + 1] - values[i - 1]) / ds;
        }
    }

    // Backward difference at end
    if base[n - 1] != base[n - 2] {
        grad[n - 1] = (values[n - 1] - values[n - 2]) / (base[n - 1] - base[n - 2]);
    }

    grad
}

/// Computes the arc length along a path defined by x and y coordinates.
///
/// * Arguments
///
/// * `x_vals` - A vector of x-coordinates.
/// * `y_vals` - A vector of y-coordinates.
///
/// * Returns a vector of arc lengths from the first point to each point.
pub fn compute_arc_length<T>(x_vals: &Vec<T>, y_vals: &Vec<T>) -> Vec<T>
where
    T: Float,
{
    let nx = x_vals.len();
    let ny = y_vals.len();
    let mut arc_lengths = match (nx, ny) {
        (0, 0) | (1, 1) => vec![T::zero(); nx],
        (n, m) if n == m => vec![T::zero(); nx],
        _ => panic!("x and y vectors must be the same length"),
    };

    if nx > 1 {
        // Forward difference for the first segment
        arc_lengths[1] = arc_lengths[0]
            + ((x_vals[1] - x_vals[0]) * (x_vals[1] - x_vals[0])
                + (y_vals[1] - y_vals[0]) * (y_vals[1] - y_vals[0]))
                .sqrt();

        // Central differences for the middle segments
        for i in 1..nx - 1 {
            let dx = x_vals[i + 1] - x_vals[i - 1];
            let dy = y_vals[i + 1] - y_vals[i - 1];
            let ds = (dx * dx + dy * dy).sqrt();
            arc_lengths[i + 1] = arc_lengths[i - 1] + ds;
        }

        // Backward difference for the last segment
        arc_lengths[nx - 1] = arc_lengths[nx - 2]
            + ((x_vals[nx - 1] - x_vals[nx - 2]) * (x_vals[nx - 1] - x_vals[nx - 2])
                + (y_vals[nx - 1] - y_vals[nx - 2]) * (y_vals[nx - 1] - y_vals[nx - 2]))
                .sqrt();
    }

    arc_lengths
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_bound_value() {
        let inside_bound = 3;
        let below_bound = -1;
        let above_bound = 10;
        let bounds = [1, 5];

        assert_eq!(
            bound_value(inside_bound, bounds[0], bounds[1]),
            inside_bound
        );

        assert_eq!(bound_value(below_bound, bounds[0], bounds[1]), bounds[0]);

        assert_eq!(bound_value(above_bound, bounds[0], bounds[1]), bounds[1]);
    }

    #[test]
    #[should_panic]
    fn test_bound_value_panic() {
        let value = 3;
        let bounds = [5, 1];

        bound_value(value, bounds[0], bounds[1]);
    }

    #[test]
    fn test_bound_polar_value() {
        let inside_bound = 0. * std::f64::consts::PI;
        let below_bound = -8. * std::f64::consts::PI;
        let above_bound = 8. * std::f64::consts::PI;
        let bounds = [-1. * std::f64::consts::PI, 1. * std::f64::consts::PI];

        assert_relative_eq!(
            bound_polar_value(inside_bound, bounds[0], bounds[1]),
            inside_bound,
            epsilon = 1e-12
        );

        assert_relative_eq!(
            bound_polar_value(below_bound, bounds[0], bounds[1]),
            0.0,
            epsilon = 1e-12
        );

        assert_relative_eq!(
            bound_polar_value(above_bound, bounds[0], bounds[1]),
            0.0,
            epsilon = 1e-12
        );
    }

    #[test]
    #[should_panic]
    fn test_bound_polar_value_panic() {
        let value = -5.;
        let bounds = [std::f64::consts::PI, -std::f64::consts::PI];

        bound_polar_value(value, bounds[0], bounds[1]);
    }

    #[test]
    fn test_calc_rectangle_points() {
        let start_point = (1.0, 1.0);
        let length_front = 3.0;
        let length_rear = 2.0;
        let width_left = 1.0;
        let width_right = 2.0;
        let heading_angle = 90.;
        let angle_units = AngleUnits::Degree;

        let calc_points = calculate_rectangle_points(
            &start_point,
            length_front,
            length_rear,
            width_left,
            width_right,
            heading_angle,
            angle_units,
        );
        let answer = [(0.0, 4.0), (3.0, 4.0), (3.0, -1.0), (0.0, -1.0)];

        println!("calc_points: {:?}", calc_points);
        println!("answer: {:?}", answer);

        for i in 0..4 {
            assert_relative_eq!(calc_points[i].0, answer[i].0, epsilon = 1e-12);
            assert_relative_eq!(calc_points[i].1, answer[i].1, epsilon = 1e-12);
        }
    }

    #[test]
    fn test_calc_line_endpoints() {
        let start_point = (1.0, 1.0);
        let length = 5.0;
        let heading_angle = 90.;
        let angle_units = AngleUnits::Degree;

        let calc_points =
            calculate_line_endpoints(&start_point, length, heading_angle, angle_units);
        let answer = [(1.0, 1.0), (1.0, 6.0)];

        println!("calc_points: {:?}", calc_points);
        println!("answer: {:?}", answer);

        for i in 0..2 {
            assert_relative_eq!(calc_points[i].0, answer[i].0, epsilon = 1e-12);
            assert_relative_eq!(calc_points[i].1, answer[i].1, epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d() {
        let values = vec![1., 2., 3., 4., 6.];
        let base = vec![0., 1., 2., 3., 4.];

        let answer = vec![1., 1., 1., 1.5, 2.];
        assert_eq!(gradient_1d(&values, &base), answer);
    }

    #[test]
    fn test_gradient_1d_linear_function() {
        let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let base = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let expected_grad = vec![1.0, 1.0, 1.0, 1.0, 1.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d_non_linear_function() {
        let values = vec![1.0, 4.0, 9.0, 16.0, 25.0];
        let base = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let expected_grad = vec![3.0, 4.0, 6.0, 8.0, 9.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d_discontinuity() {
        let values = vec![1.0, 2.0, 3.0, 10.0, 5.0];
        let base = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let expected_grad = vec![1.0, 1.0, 4.0, 1.0, -5.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d_repeated_value() {
        let values = vec![1.0, 2.0, 2.0, 3.0, 4.0];
        let base = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let expected_grad = vec![1.0, 0.5, 0.5, 1.0, 1.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d_empty_input() {
        let values: Vec<f64> = vec![];
        let base: Vec<f64> = vec![];
        let expected_grad: Vec<f64> = vec![];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad, expected_grad);
    }

    #[test]
    fn test_gradient_1d_single_point() {
        let values = vec![1.0];
        let base = vec![0.0];
        let expected_grad = vec![0.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_gradient_1d_two_points() {
        let values = vec![1.0, 2.0];
        let base = vec![0.0, 1.0];
        let expected_grad = vec![1.0, 1.0];
        let grad = gradient_1d(&values, &base);
        assert_eq!(grad.len(), expected_grad.len());
        for i in 0..grad.len() {
            assert_relative_eq!(grad[i], expected_grad[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_compute_arc_length_empty_input() {
        let x_vals: Vec<f64> = vec![];
        let y_vals: Vec<f64> = vec![];
        let result = compute_arc_length(&x_vals, &y_vals);
        assert_eq!(result, vec![]);
    }

    #[test]
    fn test_compute_arc_length_single_point() {
        let x_vals = vec![1.0];
        let y_vals = vec![2.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        assert_eq!(result, vec![0.0]);
    }

    #[test]
    fn test_compute_arc_length_two_points() {
        let x_vals = vec![1.0, 2.0];
        let y_vals = vec![2.0, 3.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        let expected = vec![0.0, 1.4142135623730951];
        for i in 0..2 {
            assert_relative_eq!(result[i], expected[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_compute_arc_length_multiple_points() {
        let x_vals = vec![1.0, 2.0, 3.0, 4.0];
        let y_vals = vec![2.0, 3.0, 4.0, 5.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        let expected = vec![
            0.0,
            1.4142135623730951,
            2.8284271247461903,
            4.242640687119284,
        ];
        for i in 0..4 {
            assert_relative_eq!(result[i], expected[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_compute_arc_length_same_x_coordinates() {
        let x_vals = vec![1.0, 1.0, 1.0];
        let y_vals = vec![2.0, 3.0, 4.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        let expected = vec![0.0, 1.0, 2.0];
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_compute_arc_length_same_y_coordinates() {
        let x_vals = vec![1.0, 2.0, 3.0];
        let y_vals = vec![2.0, 2.0, 2.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        let expected = vec![0.0, 1.0, 2.0];
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_compute_arc_length_large_values() {
        let x_vals = vec![1000.0, 2000.0, 3000.0];
        let y_vals = vec![2000.0, 3000.0, 4000.0];
        let result = compute_arc_length(&x_vals, &y_vals);
        let expected = vec![0.0, 1414.2135623730951, 2828.4271247461903];
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i], epsilon = 1e-12);
        }
    }
}
