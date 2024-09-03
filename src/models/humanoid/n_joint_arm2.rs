// rust robotics
use crate::models::base::{self, System, SystemH};
use crate::utils::files;

// 3rd party or std
extern crate nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Model<T, const N: usize, const M: usize> {
    pub link_lengths: Vec<T>,
}

impl<T, const N: usize, const M: usize> Model<T, N, M> {
    fn calculate_a(&self) -> na::SMatrix<T, N, N> {
        todo!()
    }

    fn distance_squared_to_goal(
        &self,
        position_current: &na::SVector<T, 2>,
        position_goal: &na::SVector<T, 2>,
    ) -> (na::SVector<T, 2>, T)
    where
        na::SVector<T, 2>: std::ops::Sub<Output = na::SVector<T, 2>>,
        T: std::ops::Add<Output = T> + std::ops::Mul<Output = T> + Copy,
    {
        let error = *position_goal - *position_current;

        let distance_squared = error[0] * error[0] + error[1] * error[1];

        (error, distance_squared)
    }
}

impl<const N: usize, const M: usize> base::SystemH<f64, N, M, 2> for Model<f64, N, M> {
    fn num_states_to_num_dim_ratio(&self) -> u8 {
        4u8
    }

    fn feasible_state_initial(&self, angles_desired: &[f64]) -> na::SVector<f64, N> {
        let ratio =
            <Model<f64, N, M> as SystemH<f64, N, M, 2>>::num_states_to_num_dim_ratio(self) as usize;
        let dof = N / ratio;

        assert_eq!(angles_desired.len(), self.link_lengths.len());
        assert_eq!(dof, self.link_lengths.len());

        let mut x = na::SVector::<f64, N>::zeros();

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

    fn inverse_kinematics(&self, position_desired: Vec<f64>, x: &na::SVector<f64, N>) -> Vec<f64> {
        let ratio = self.num_states_to_num_dim_ratio() as usize;
        let dof = N / ratio;
        assert_eq!(position_desired.len(), 2);
        assert_eq!(dof, M);

        let mut pos_jac_vec: Vec<f64> = vec![0.0; N / (ratio / 2)];
        let mut angles_vec: Vec<f64> = vec![0.0; N / ratio];

        let mut angle_sum = 0.;

        for i in 0..dof {
            println!("i: {}", i);
            angle_sum += x[ratio * i + 2];

            for j in i..dof {
                pos_jac_vec[2 * i] -= self.link_lengths[j] * f64::sin(angle_sum);
                pos_jac_vec[2 * i + 1] += self.link_lengths[j] * f64::cos(angle_sum);
            }

            angles_vec[i] = x[ratio * i + 2];
        }

        let position_jacobian: na::DMatrix<f64> = na::DMatrix::from_vec(2, N / ratio, pos_jac_vec);

        // TODO: no unwrap
        let jacobian_inverse = position_jacobian.pseudo_inverse(1e-12).unwrap();

        let (error, distance_squared) = self.distance_squared_to_goal(
            &na::SVector::<f64, 2>::new(x[ratio * dof - 4], x[ratio * dof - 3]),
            &na::SVector::<f64, 2>::new(position_desired[0], position_desired[1]),
        );

        let joint_angle_goals = jacobian_inverse.clone() * error;

        println!("error: {}, jacobian_inverse: {}", error, jacobian_inverse);

        println!("joint angles: {}", joint_angle_goals);

        joint_angle_goals.data.as_vec().to_vec()
    }
}

/// N joint arm 2D
///
/// All the states are in the global frame
///
/// x_dot = [joint1_x_dot, joint1_y_dot, joint1_theta_dot, joint1_theta_ddot, ..., jointN_x_dot, jointN_y_dot, jointN_theta_dot, jointN_theta_ddot]
/// x = [joint1_x, joint1_y, joint1_theta, joint1_theta_dot, ..., jointN_x, jointN_y, jointN_theta, jointN_theta_dot]
/// u = [torque1, ... torqueN], torque is just angular acceleration in this instance because this is kinematic model
impl<const N: usize, const M: usize> base::System<f64, N, M> for Model<f64, N, M> {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, N>,
        u: &nalgebra::SVector<f64, M>,
        _t: f64,
    ) -> na::SVector<f64, N> {
        let ratio =
            <Model<f64, N, M> as SystemH<f64, N, M, 2>>::num_states_to_num_dim_ratio(self) as usize;
        let remainder = N % ratio;
        let dof = N / ratio;

        // number of states should be divisible by the ratio
        // Degrees of freedom should match the number links as well as number of inputs
        assert_eq!(remainder, 0usize);
        assert_eq!(dof, self.link_lengths.len());
        assert_eq!(dof, M);

        let mut x_dot = na::SVector::<f64, N>::zeros();

        for i in 0..dof {
            for j in 0..=i {
                x_dot[ratio * i] +=
                    -self.link_lengths[j] * f64::sin(x[ratio * j + 2]) * (x[ratio * j + 3] + u[j]);
                x_dot[ratio * i + 1] +=
                    self.link_lengths[j] * f64::cos(x[ratio * j + 2]) * (x[ratio * j + 3] + u[j]);
            }
            x_dot[ratio * i + 2] = x[ratio * i + 3] + u[i];
            x_dot[ratio * i + 3] = 0.0;

            // if i == 0 {
            //     x_dot[i] = -self.link_lengths[i] * f64::sin(x[i + 2]) * (x[i + 3] + u[i]);
            //     x_dot[i + 1] = self.link_lengths[i] * f64::cos(x[i + 2]) * (x[i + 3] + u[i]);
            //     x_dot[i + 2] = x[ratio * i + 3] + u[i];
            //     x_dot[i + 3] = 0.0;
            // } else {
            //     x_dot[ratio * i] = x_dot[ratio * (i - 1)]
            //         - self.link_lengths[i]
            //             * f64::sin(angle_sum)
            //             * (x[ratio * (i - 1) + 3] + u[i - 1])
            //         - (self.link_lengths[i] * f64::sin(angle_sum) * (x[ratio * i + 3] + u[i]));
            //     x_dot[ratio * i + 1] = x_dot[ratio * (i - 1) + 1]
            //         + self.link_lengths[i]
            //             * f64::cos(angle_sum)
            //             * (x[ratio * (i - 1) + 3] + u[i - 1])
            //         + (self.link_lengths[i] * f64::cos(angle_sum) * (x[ratio * i + 3] + u[i]));
            //     x_dot[ratio * i + 2] = x[ratio * i + 3] + u[i];
            //     x_dot[ratio * i + 3] = 0.0;
            // }
        }
        // println!("x_dot: {}", x_dot);

        x_dot
    }

    #[allow(unused)]
    fn calculate_jacobian(
        &self,
        x: &na::SVector<f64, N>,
        u: &na::SVector<f64, M>,
        _t: f64,
    ) -> (na::SMatrix<f64, N, N>, na::SMatrix<f64, N, M>) {
        // (
        //     Model::calculate_a(&self, x, u),
        //     Model::calculate_b(&self, x, u),
        // )
        todo!();
    }

    #[allow(unused)]
    fn calculate_input(
        &self,
        _x: &nalgebra::SVector<f64, N>,
        _x_dot: &nalgebra::SVector<f64, N>,
        _t: f64,
    ) -> nalgebra::SVector<f64, M> {
        todo!()
    }

    #[allow(unused)]
    fn read(&mut self, filename: &str) -> () {
        todo!()
        // let data: Model = files::read_config(filename);
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
