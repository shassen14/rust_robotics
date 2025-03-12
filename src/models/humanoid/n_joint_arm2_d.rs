use core::num;

// rust robotics
use crate::models::base_d::{self, SystemHD};
use crate::utils::files;

// 3rd party or std
use nalgebra as na;
use num_traits;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct ModelD<T> {
    pub link_lengths: Vec<T>,
}

impl<T> ModelD<T> {
    fn distance_squared_to_goal(
        &self,
        position_current: &na::DVector<T>,
        position_goal: &na::DVector<T>,
    ) -> (na::DVector<T>, T)
    where
        T: num_traits::Float,
        na::DVector<T>: std::ops::Sub<Output = na::DVector<T>>,
    {
        // Ensure the positions are 2D (x, y)
        assert!(position_current.len() == 2);
        assert!(position_goal.len() == 2);

        // TODO: somehow remove the clone? Not super necessary since they're 2x1 vectors
        let error = position_goal.clone() - position_current.clone();

        let distance_squared = error[0] * error[0] + error[1] * error[1];

        (error, distance_squared)
    }
}

impl base_d::SystemHD<f64, 2> for ModelD<f64> {
    fn num_states_to_num_dim_ratio(&self) -> u8 {
        4u8
    }

    fn calculate_feasible_state_initial(&self, angles_desired: &[f64]) -> na::DVector<f64> {
        assert_eq!(angles_desired.len(), self.link_lengths.len());
        let dof = self.link_lengths.len();

        let ratio = <ModelD<f64> as SystemHD<f64, 2>>::num_states_to_num_dim_ratio(self) as usize;

        let mut x = na::DVector::<f64>::zeros(dof * ratio);

        let mut angle_sum = 0.;

        for i in 0..dof {
            angle_sum += angles_desired[i];
            if i == 0 {
                x[ratio * i] = self.link_lengths[i] * f64::cos(angles_desired[i]);
                x[ratio * i + 1] = self.link_lengths[i] * f64::sin(angles_desired[i]);
                x[ratio * i + 2] = angle_sum;
            } else {
                x[ratio * i] = x[ratio * (i - 1)] + self.link_lengths[i] * f64::cos(angle_sum);
                x[ratio * i + 1] =
                    x[ratio * (i - 1) + 1] + self.link_lengths[i] * f64::sin(angle_sum);
                x[ratio * i + 2] = angle_sum;
            }
        }
        x
    }

    fn inverse_kinematics(&self, position_desired: Vec<f64>, x: &na::DVector<f64>) -> Vec<f64> {
        assert_eq!(position_desired.len(), 2);
        let ratio = self.num_states_to_num_dim_ratio() as usize;
        let dof = self.link_lengths.len();
        let num_states = dof * ratio;

        let mut pos_jac_vec: Vec<f64> = vec![0.0; num_states / (ratio / 2)];
        let mut angles_vec: Vec<f64> = vec![0.0; num_states / ratio];

        for i in 0..dof {
            for j in i..dof {
                pos_jac_vec[2 * i] -= self.link_lengths[j] * f64::sin(x[ratio * i + 2]);
                pos_jac_vec[2 * i + 1] += self.link_lengths[j] * f64::cos(x[ratio * i + 2]);
            }

            angles_vec[i] = x[ratio * i + 2];
        }

        let position_jacobian: na::DMatrix<f64> =
            na::DMatrix::from_vec(2, num_states / ratio, pos_jac_vec);

        // TODO: no unwrap
        let jacobian_inverse = position_jacobian.pseudo_inverse(1e-12).unwrap();

        let (error, _) = self.distance_squared_to_goal(
            &na::DVector::<f64>::from_vec(vec![x[ratio * dof - 4], x[ratio * dof - 3]]),
            &na::DVector::<f64>::from_vec(vec![position_desired[0], position_desired[1]]),
        );

        let joint_angle_difference = jacobian_inverse.clone() * error;

        joint_angle_difference.data.as_vec().to_vec()
    }
}

/// N joint arm 2D
///
/// All the states are in the global frame
///
/// x_dot = [joint1_x_dot, joint1_y_dot, joint1_theta_dot, joint1_theta_ddot, ..., jointN_x_dot, jointN_y_dot, jointN_theta_dot, jointN_theta_ddot]
/// x = [joint1_x, joint1_y, joint1_theta, joint1_theta_dot, ..., jointN_x, jointN_y, jointN_theta, jointN_theta_dot]
/// u = [torque1, ... torqueN], torque is just angular acceleration in this instance because this is kinematic model
impl base_d::SystemD<f64> for ModelD<f64> {
    fn get_derivatives(
        &self,
        x: &nalgebra::DVector<f64>, // N x 1
        u: &nalgebra::DVector<f64>, // M x 1
        _t: f64,
    ) -> na::DVector<f64> {
        let ratio = <ModelD<f64> as SystemHD<f64, 2>>::num_states_to_num_dim_ratio(self) as usize;
        let num_states = x.len();
        let remainder = num_states % ratio;
        let dof = num_states / ratio;

        // Number of states should be divisible by the ratio
        // Degrees of freedom should match the number links as well as number of inputs
        assert_eq!(remainder, 0usize);
        assert_eq!(dof, self.link_lengths.len());
        assert_eq!(dof, u.len());

        let mut x_dot = na::DVector::<f64>::zeros(num_states);

        for i in 0..dof {
            for j in 0..=i {
                x_dot[ratio * i] +=
                    -self.link_lengths[j] * f64::sin(x[ratio * j + 2]) * (x[ratio * j + 3] + u[j]);
                x_dot[ratio * i + 1] +=
                    self.link_lengths[j] * f64::cos(x[ratio * j + 2]) * (x[ratio * j + 3] + u[j]);
            }
            x_dot[ratio * i + 2] = x[ratio * i + 3] + u[i];
            x_dot[ratio * i + 3] = 0.0;
        }

        x_dot
    }

    #[allow(unused)]
    fn calculate_jacobian(
        &self,
        x: &na::DVector<f64>,
        u: &na::DVector<f64>,
        _t: f64,
    ) -> (na::DMatrix<f64>, na::DMatrix<f64>) {
        todo!();
    }

    #[allow(unused)]
    fn calculate_input(
        &self,
        _x: &nalgebra::DVector<f64>,
        _x_dot: &nalgebra::DVector<f64>,
        _t: f64,
    ) -> nalgebra::DVector<f64> {
        todo!()
    }

    #[allow(unused)]
    fn read(&mut self, filename: &str) -> () {
        let data: ModelD<f64> = files::read_toml(filename);
        self.link_lengths = data.link_lengths;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn random_test() {
        let mut x = na::SVector::<f64, 12>::zeros();
        x[2] = std::f64::consts::PI;
        x[6] = std::f64::consts::PI;
        x[10] = std::f64::consts::PI;

        let ratio = 4usize;

        let mut angle_sum = 0.;
        for i in 0..=2 {
            angle_sum += x[ratio * i + 2];
        }

        println!("{}", angle_sum);
        assert_eq!(angle_sum, 3.0 * std::f64::consts::PI);
    }
}
