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

    /// Calculates total length
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * Returns the length from the rear wheels (middle) to the center of gravity
    fn calculate_length_total(&self) -> f64 {
        self.length_front + self.length_rear
    }

    /// Calculates sideslip angle which is the angle between the wheel direction and
    /// the vehicle direction
    ///
    /// * Arguments
    ///
    /// * `self` - Model's parameters
    /// * `steer_angle` - the road wheel angle the wheels are facing relative the the vehicle direction
    /// * Returns the length from the rear wheels (middle) to the center of gravity
    fn calculate_sideslip(&self, steer_angle: f64) -> f64 {
        f64::atan(self.length_rear * f64::tan(steer_angle) / self.calculate_length_total())
    }

    /// TODO: Derivative of the nonlinear model is slightly difficult. Need to figure out how to take partial derivatives
    /// with different frames easily. Might add an enum for it?
    #[allow(unused)]
    fn calculate_f(&self, steer_angle: f64) -> na::SMatrix<f64, 3, 3> {
        na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
        ]));
        // let x1
        todo!();
    }
}

/// Bicycle Kinematic Model with 3 states and 2 inputs
///
/// x_dot = [pos_x_dot, pos_y_dot, heading_dot], global frame
/// u = [vel_x, steering_angle], body frame
impl base::System<f64, 3, 2> for Model {
    /// Derivative function x_dot = f(x, u, t) which is a function of
    /// the system's current state, control input, and time
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * Returns rate of change of the system's states
    ///
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 3>,
        u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> na::SVector<f64, 3> {
        let sideslip: f64 = self.calculate_sideslip(u[1]);

        // x_dot = vel_x * cos(yaw + sideslip)
        // y_dot = vel_x * sin(yaw + sideslip)
        // yaw_dot = vel_x * tan(steer_angle) * cos(sideslip) / length_total
        let pos_x_dot: f64 = u[0] * f64::cos(x[2] + sideslip);
        let pos_y_dot: f64 = u[0] * f64::sin(x[2] + sideslip);
        let yaw_dot: f64 =
            u[0] * f64::tan(u[1]) * f64::cos(sideslip) / self.calculate_length_total();
        na::SVector::<f64, 3>::new(pos_x_dot, pos_y_dot, yaw_dot)
    }

    /// Get system's jacobian with respect to state and input partial derivatives
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * Returns a pair of matrices where the first is the partial derivative of f(x, u, t)
    /// with respect to each state (x), and the second is the partial derivative of f(x, u, t)
    /// with respect to each control input (u). [(NxN), (NxM)] since there are n states and
    /// M inputs.
    ///
    fn get_jacobian(
        &self,
        _x: &nalgebra::SVector<f64, 3>,
        u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> (na::SMatrix<f64, 3, 3>, na::SMatrix<f64, 3, 2>) {
        (
            Model::calculate_f(&Model::new(self.length_front, self.length_rear), u[1]),
            na::SMatrix::<f64, 2, 2>::zeros(),
        );
        todo!();
    }

    /// Read a toml file to change the model's parameters if any
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, and values that could be changed from the toml file
    /// * `filename` - File name to read values
    ///
    fn read(&mut self, filename: &str) -> () {
        let data: Model = files::read_config(filename);

        self.length_front = data.length_front;
        self.length_rear = data.length_rear;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use core::f64;

    use super::*;
    use crate::num_methods::runge_kutta;

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
        let steer0: f64 = f64::consts::FRAC_PI_8;

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
}
