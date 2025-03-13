// rust robotics
use crate::models::base;
use crate::utils::files;

// 3rd party or std
extern crate nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Model {
    length_front: f64,
    length_rear: f64,
}

impl Model {
    /// Create a new Kinematic Bicycle Model given two parameters
    ///
    /// * Arguments
    ///
    /// * `length_front` - length from the front wheels (middle) to the center of gravity
    /// * `length_rear` - length from the rear wheels (middle) to the center of gravity
    /// * Returns Model object
    pub fn new(length_front: f64, length_rear: f64) -> Self {
        Model {
            length_front: length_front,
            length_rear: length_rear,
        }
    }

    /// Gets the length from the front wheels (middle) to the center of gravity
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * Returns the length from the front wheels (middle) to the center of gravity
    pub fn get_length_front(&self) -> f64 {
        self.length_front
    }

    /// Gets the length from the rear wheels (middle) to the center of gravity
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * Returns the length from the rear wheels (middle) to the center of gravity
    pub fn get_length_rear(&self) -> f64 {
        self.length_rear
    }

    /// Calculates total length, wheel base
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * Returns the length from the rear wheels (middle) to the center of gravity
    pub fn get_wheelbase(&self) -> f64 {
        self.length_front + self.length_rear
    }

    /// Calculates sideslip angle which is the angle between the wheel direction and
    /// the vehicle direction
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * `road_wheel_angle` - the road wheel angle the wheels are facing relative the the vehicle direction
    /// * Returns the length from the rear wheels (middle) to the center of gravity
    ///
    fn calculate_sideslip(&self, road_wheel_angle: f64) -> f64 {
        // beta = atan(l_r * tan(rwa) / wheel_base)
        f64::atan(self.length_rear * f64::tan(road_wheel_angle) / self.get_wheelbase())
    }

    // TODO: calculate a and calculate b seem to be right? Confirmed with online calculators
    // Simulation seem to be way off unfortunately. Need to reevaluate this
    #[allow(unused)]
    fn calculate_a(
        &self,
        x: &na::SVector<f64, 3>,
        u: &na::SVector<f64, 2>,
    ) -> na::SMatrix<f64, 3, 3> {
        let sideslip: f64 = self.calculate_sideslip(u[1]);

        // df0/ dx0, df0/ dx1, df0 / dx2
        // df1/ dx0, df1/ dx1, df1 / dx2
        // df2/ dx0, df2/ dx1, df2 / dx2
        // where
        // f0: x_dot = vel_x * cos(yaw + sideslip)
        // f1: y_dot = vel_x * sin(yaw + sideslip)
        // f2: yaw_dot = vel_x * tan(road_wheel_angle) * cos(sideslip) / length_total

        let df0x0 = 0f64;
        let df0x1 = 0f64;
        let df0x2 = -u[0] * f64::sin(x[2] + sideslip);

        let df1x0 = 0f64;
        let df1x1 = 0f64;
        let df1x2 = u[0] * f64::cos(x[2] + sideslip);

        let df2x0 = 0f64;
        let df2x1 = 0f64;
        let df2x2 = 0f64;

        // inputs by columns
        na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [df0x0, df1x0, df2x0],
            [df0x1, df1x1, df2x1],
            [df0x2, df1x2, df2x2],
        ]))
    }

    #[allow(unused)]
    fn calculate_b(
        &self,
        x: &na::SVector<f64, 3>,
        u: &na::SVector<f64, 2>,
    ) -> na::SMatrix<f64, 3, 2> {
        let length_total = self.get_wheelbase();
        let sideslip = self.calculate_sideslip(u[1]);
        // df0/ du0, df0/ du1,
        // df1/ du0, df1/ du1,
        // df2/ du0, df2/ du1,
        // where
        // f0: x_dot = vel_x * cos(yaw + sideslip)
        // f1: y_dot = vel_x * sin(yaw + sideslip)
        // f2: yaw_dot = vel_x * tan(road_wheel_angle) * cos(sideslip) / length_total
        let df0u0 = f64::cos(x[2] + sideslip);
        let df1u0 = f64::sin(x[2] + sideslip);

        let df2u0_denom = length_total
            * f64::sqrt(
                f64::powi(self.length_rear, 2) * f64::powi(f64::tan(u[1]), 2)
                    / f64::powi(length_total, 2)
                    + 1.,
            );

        // let df2u0 = f64::tan(u[1]) * f64::cos(sideslip) / length_total;
        let df2u0 = f64::tan(u[1]) / df2u0_denom;

        // derivative of sideslip wrt road wheel angle for the first two terms.
        // coefficient is l_r * t * sec^2(u[1])
        let dfu1_coeff = self.length_rear * length_total / (f64::cos(u[1]) * f64::cos(u[1]));

        // derivative of sideslip wrt road wheel angle for the first two terms.
        // denominator = l_r^2 * tan^2(u[1]) + l_t^2
        let dfu1_denom = (self.length_rear * self.length_rear) * (f64::tan(u[1]) * f64::tan(u[1]))
            + (length_total * length_total);

        // // numerator for last term
        // let num = length_total * u[0] / (f64::cos(u[1]) * f64::cos(u[1]));
        // // denominator for last term
        // let den = ((self.length_rear * self.length_rear) * (f64::tan(u[1]) * f64::tan(u[1]))
        //     + (length_total * length_total))
        //     * f64::sqrt(
        //         (self.length_rear * self.length_rear) * (f64::tan(u[1]) * f64::tan(u[1]))
        //             / (length_total * length_total)
        //             + 1.0,
        //     );

        // numerator for last term
        let df2u1_num = f64::powf(length_total, 3f64) * u[0] / (f64::cos(u[1]) * f64::cos(u[1]));
        // denominator for last term
        let df2u1_denom = f64::powf(
            (self.length_rear * self.length_rear) * (f64::tan(u[1]) * f64::tan(u[1]))
                + (length_total * length_total),
            1.5f64,
        ) * length_total.abs();

        let df0u1 = -u[0] * dfu1_coeff * f64::sin(x[2] + sideslip) / dfu1_denom;
        let df1u1 = u[0] * dfu1_coeff * f64::cos(x[2] + sideslip) / dfu1_denom;
        let df2u1 = df2u1_num / df2u1_denom;

        // input by column
        na::SMatrix::<f64, 3, 2>::from_array_storage(na::ArrayStorage([
            [df0u0, df1u0, df2u0],
            [df0u1, df1u1, df2u1],
        ]))
    }
}

/// Bicycle Kinematic Model with 3 states and 2 inputs
///
/// x_dot = [pos_x_dot, pos_y_dot, heading_dot], global frame
/// x = [pos_x, pos_y, heading]
/// u = [vel_x, road_wheel_angle], body frame
impl base::System<f64, 3, 2> for Model {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 3>,
        u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> na::SVector<f64, 3> {
        let sideslip: f64 = self.calculate_sideslip(u[1]);

        // x_dot = vel_x * cos(yaw + sideslip)
        // y_dot = vel_x * sin(yaw + sideslip)
        // yaw_dot = vel_x * tan(road_wheel_angle) * cos(sideslip) / length_total
        let pos_x_dot: f64 = u[0] * f64::cos(x[2] + sideslip);
        let pos_y_dot: f64 = u[0] * f64::sin(x[2] + sideslip);
        let yaw_dot: f64 = u[0] * f64::tan(u[1]) * f64::cos(sideslip) / self.get_wheelbase();
        na::SVector::<f64, 3>::new(pos_x_dot, pos_y_dot, yaw_dot)
    }

    fn calculate_jacobian(
        &self,
        x: &na::SVector<f64, 3>,
        u: &na::SVector<f64, 2>,
        _t: f64,
    ) -> (na::SMatrix<f64, 3, 3>, na::SMatrix<f64, 3, 2>) {
        (
            Model::calculate_a(&self, x, u),
            Model::calculate_b(&self, x, u),
        )
    }

    fn calculate_input(
        &self,
        _x: &nalgebra::SVector<f64, 3>,
        _x_dot: &nalgebra::SVector<f64, 3>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 2> {
        todo!()
    }

    fn read(&mut self, filename: &str) -> () {
        let data: Model = files::read_toml(filename);

        self.length_front = data.length_front;
        self.length_rear = data.length_rear;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {

    use super::*;
    use crate::num_methods::runge_kutta;
    use approx::assert_relative_eq;
    use base::System;

    #[test]
    fn bicycle_kinem_prop_straight() {
        let lf: f64 = 1.0;
        let lr: f64 = 1.0;
        let veh: Model = Model::new(lf, lr);

        let x0: f64 = 0.;
        let y0: f64 = 0.;
        let yaw0: f64 = 0.;

        let vel0: f64 = 3.;
        let steer0: f64 = 0.;

        let start: f64 = 0.;
        let end: f64 = 10.;
        let step: f64 = 0.01;
        let total_time = end - start;

        let mut t0 = start;
        let mut tf = t0 + step;

        let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(x0, y0, yaw0);
        let input0: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(vel0, steer0);

        let mut result = state0;
        let expected_result: na::SVector<f64, 3> =
            na::SVector::<f64, 3>::new(x0 + vel0 * total_time, 0., 0.);

        while tf <= end {
            result = base::System::propagate(&veh, &result, &input0, t0, step, runge_kutta::rk4);
            t0 = tf;
            tf += step;
        }

        let max_error: f64 = 1e-11;
        let total_error = expected_result - result;

        println!("Result: {},  Expected Result: {}", result, expected_result);
        for i in 0..3 {
            println!(
                "i: {},  total error: {},  max error: {}\n",
                i,
                total_error[i].abs(),
                max_error.abs()
            );
            assert!(total_error[i].abs() < max_error.abs())
        }
    }

    #[test]
    fn bicycle_kinem_prop_turn() {
        let lf: f64 = 1.0;
        let lr: f64 = 1.0;
        let veh: Model = Model::new(lf, lr);

        let x0: f64 = 0.;
        let y0: f64 = x0;
        let yaw0: f64 = 0.;

        let vel0: f64 = 5.;
        let steer0: f64 = std::f64::consts::FRAC_PI_8;

        let start: f64 = 0.;
        let end: f64 = 10.;
        let step: f64 = 0.01;
        let total_time = end - start;

        let mut t0 = start;
        let mut tf = t0 + step;

        let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(x0, y0, yaw0);
        let input0: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(vel0, steer0);

        let mut result = state0;
        let max_result = x0 + vel0 * total_time;
        let min_result = -max_result;

        while tf <= end {
            result = base::System::propagate(&veh, &result, &input0, t0, step, runge_kutta::rk4);
            t0 = tf;
            tf += step;
        }

        println!(
            "Result: {}, min_result: {}, max_result: {}",
            result, min_result, max_result
        );

        // Not the best assertion, but the values have to be in between the maximum length from going straight and the negative of that value
        for i in 0..3 {
            assert!(result[i] > min_result);
            assert!(result[i] < max_result);
        }
    }

    #[test]
    fn bicycle_kinem_jacobian() {
        // TODO: need to check the 2nd jacobian for B matrix

        let lf: f64 = 1.0;
        let lr: f64 = 1.0;
        let veh: Model = Model::new(lf, lr);

        let x0: f64 = 0.;
        let y0: f64 = x0;
        let yaw0: f64 = 0.;

        let vel0: f64 = 4.;
        let steer0: f64 = std::f64::consts::FRAC_PI_8 * 1.0;

        let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(x0, y0, yaw0);
        let input0: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(vel0, steer0);

        let jacobian = veh.calculate_jacobian(&state0, &input0, 0.);

        // the magnitude of x_dot and y_dot combined should equal to forward velocity
        // this model assumes lateral velocity ~ 0
        let calc1 = f64::sqrt(
            jacobian.0[(0, 2)] * jacobian.0[(0, 2)] + jacobian.0[(1, 2)] * jacobian.0[(1, 2)],
        );
        assert_relative_eq!(calc1, vel0, epsilon = 1e-12);
    }
}
