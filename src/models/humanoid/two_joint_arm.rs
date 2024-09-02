// rust robotics
use crate::models::base;
use crate::utils::files;
use crate::utils::math;

// 3rd party or std
extern crate nalgebra as na;
use serde::{Deserialize, Serialize};

// Test file to figure out patterns in order to do this more generically for at least 2D arms

#[derive(Debug, Deserialize, Serialize)]
pub struct Model<T> {
    pub link_lengths: [T; 2],
}

impl Model<f64> {
    pub fn feasible_state_initial(&self, angle_desired: [f64; 2]) -> na::SVector<f64, 8> {
        let mut x0: na::SVector<f64, 8> = na::SVector::<f64, 8>::zeros();
        x0[0] = self.link_lengths[0] * f64::cos(angle_desired[0]);
        x0[1] = self.link_lengths[0] * f64::sin(angle_desired[0]);
        x0[2] = angle_desired[0];
        x0[4] = x0[0] + self.link_lengths[1] * f64::cos(angle_desired[0] + angle_desired[1]);
        x0[5] = x0[1] + self.link_lengths[1] * f64::sin(angle_desired[0] + angle_desired[1]);
        x0[6] = angle_desired[1];
        x0
    }
    pub fn inverse_kinematics(
        &self,
        position_desired: &na::SVector<f64, 2>,
    ) -> na::SVector<f64, 2> {
        // let link_vec = na::DVector::from_vec(self.link_lengths.to_vec());
        let mut x_desired = position_desired[0];
        let mut y_desired = position_desired[1];
        let norm_desired = position_desired.norm();
        let max_distance = self.link_lengths[0] + self.link_lengths[1] - 0.0001;

        if norm_desired >= max_distance {
            x_desired = position_desired[0] * max_distance / norm_desired;
            y_desired = position_desired[1] * max_distance / norm_desired;
        }

        let position_desired_final = na::SVector::<f64, 2>::new(x_desired, y_desired);
        let theta2_goal = f64::acos(
            (position_desired_final.norm_squared()
                - f64::powi(self.link_lengths[0], 2)
                - f64::powi(self.link_lengths[1], 2))
                / (2. * self.link_lengths[0] * self.link_lengths[1]),
        );

        let tmp = f64::atan2(
            self.link_lengths[1] * f64::sin(theta2_goal),
            self.link_lengths[0] + self.link_lengths[1] * f64::cos(theta2_goal),
        );
        let theta1_goal = f64::atan2(y_desired, x_desired) - tmp;

        na::SVector::<f64, 2>::new(theta1_goal, theta2_goal)
    }

    // TODO: most likely this is for an outside controller
    pub fn calculate_x_dot_desired(
        &self,
        theta_desired: &na::SVector<f64, 2>,
        x: &na::SVector<f64, 8>,
    ) -> na::SVector<f64, 8> {
        let theta1_dif = math::bound_polar_value(
            theta_desired[0] - x[2],
            -std::f64::consts::PI,
            std::f64::consts::PI,
        );
        let theta2_dif = math::bound_polar_value(
            theta_desired[1] - x[6],
            -std::f64::consts::PI,
            std::f64::consts::PI,
        );
        let mut x_dot_desired = na::SVector::<f64, 8>::zeros();
        x_dot_desired[3] = theta1_dif;
        x_dot_desired[7] = theta2_dif;
        x_dot_desired
    }
}

/// x_dot = [joint1_x_dot, joint1_y_dot, joint1_theta_dot, joint1_theta_ddot joint2_x_dot, joint2_y_dot, joint2_theta_dot, joint2_theta_ddot]
/// x = [joint1_x, joint1_y, joint1_theta, joint1_theta_dot joint2_x, joint2_y, joint2_theta, joint2_theta_dot]
/// u = [torque1, torque2], torque is just angular acceleration in this instance because this is kinematic model
/// i.e. torque = moment of inertia * angular acceleration, inertia = 1
impl base::System<f64, 8, 2> for Model<f64> {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 8>,
        u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> na::SVector<f64, 8> {
        assert_eq!(4, 8 / self.link_lengths.len());

        let mut x_dot = na::SVector::<f64, 8>::zeros();

        // joint 1
        x_dot[0] = -self.link_lengths[0] * f64::sin(x[2]) * (x[3] + u[0]);
        x_dot[1] = self.link_lengths[0] * f64::cos(x[2]) * (x[3] + u[0]);
        x_dot[2] = x[3] + u[0];
        x_dot[3] = 0.0;

        // joint 2
        x_dot[4] = (-self.link_lengths[0] * f64::sin(x[2])
            - self.link_lengths[1] * f64::sin(x[2] + x[6]))
            * (x[3] + u[0])
            + (-self.link_lengths[1] * f64::sin(x[2] + x[6])) * (x[7] + u[1]);
        x_dot[5] = (self.link_lengths[0] * f64::cos(x[2])
            + self.link_lengths[1] * f64::cos(x[2] + x[6]))
            * (x[3] + u[0])
            + (self.link_lengths[1] * f64::cos(x[2] + x[6])) * (x[7] + u[1]);
        x_dot[6] = x[7] + u[1];
        x_dot[7] = 0.0;

        x_dot
    }

    fn calculate_jacobian(
        &self,
        x: &na::SVector<f64, 8>,
        u: &na::SVector<f64, 2>,
        _t: f64,
    ) -> (na::SMatrix<f64, 8, 8>, na::SMatrix<f64, 8, 2>) {
        // (
        //     Model::calculate_a(&self, x, u),
        //     Model::calculate_b(&self, x, u),
        // )
        todo!();
    }

    fn calculate_input(
        &self,
        _x: &nalgebra::SVector<f64, 8>,
        x_dot_desired: &nalgebra::SVector<f64, 8>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 2> {
        na::SMatrix::<f64, 2, 2>::new(0.025, 0., 0., 0.1)
            * na::SVector::<f64, 2>::new(x_dot_desired[3], x_dot_desired[7])
    }

    fn read(&mut self, filename: &str) -> () {
        // let data: Model = files::read_config(filename);
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
}
