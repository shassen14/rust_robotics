// rust robotics
use crate::models::base;
use crate::utils::constant;
use crate::utils::files;

// 3rd party or std
extern crate nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Model {
    mass: f64,
    area_frontal: f64,
}

impl Model {
    /// Create a new Longitudinal Dynamics Model given two parameters
    ///
    /// * Arguments
    ///
    /// * `mass` - mass of the vehicle
    /// * `area_frontal` - frontal area of the vehicle
    /// * Returns Model object
    pub fn new(mass: f64, area_frontal: f64) -> Self {
        assert!(mass > 0.0, "mass must be positive");
        assert!(area_frontal > 0.0, "area_frontal must be positive");

        Model {
            mass: mass,
            area_frontal: area_frontal,
        }
    }

    /// Calculates force due to aerodynamic drag [N]
    ///
    /// * Arguments
    ///
    /// * `velocity` - the velocity of the vehicle [m/s]
    /// * Returns the force due to aerodynamic drag [N]
    fn calc_f_aero(&self, velocity: f64) -> f64 {
        0.5 * constant::air_density::<f64>()
            * self.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity
    }

    /// Calculates the force due to rolling resistance [N]
    ///
    /// * Arguments
    ///
    /// * `grade_angle` - Grade angle of the surface [radians]
    /// * Returns the force due to rolling resistance [N]
    fn calc_f_roll(&self, grade_angle: f64) -> f64 {
        self.mass
            * constant::gravity::<f64>()
            * constant::rolling_resistance::<f64>()
            * f64::cos(grade_angle)
    }

    /// Calculates force due to grade resistance [N]
    ///
    /// * Arguments
    ///
    /// * `grade_angle` - Grade angle of the surface [radians]
    /// * Returns the force due to grade resistance [N]
    fn calc_f_grade(&self, grade_angle: f64) -> f64 {
        self.mass * constant::gravity::<f64>() * f64::sin(grade_angle)
    }
}

/// TODO: technically we could go reverse if f_brake is greater than everything else
/// TODO: have a reverse state/model to go backwards
/// TODO: include grade road angle

/// Longitudinal dynamics model of a car
///
/// x_dot = [vel_x_dot], vehicle frame
/// x = [vel_x]
/// u = [F], F_engine/tractive > 0 | F_brake < 0
impl base::System<f64, 1, 1> for Model {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 1>,
        u: &nalgebra::SVector<f64, 1>,
        _t: f64,
    ) -> na::SVector<f64, 1> {
        // F_aero = 0.5 * air_density * frontal_area * drag_coeff * x[0]^2
        // F_roll = mass * gravity * rolling resistance * cos(grade angle)
        // F_grade = mass * gravity * sin(grade angle)
        // vel_x_dot = (u[0] - F_aero) / m
        let f_aero: f64 = self.calc_f_aero(x[0]);
        let f_roll: f64 = self.calc_f_roll(0.);
        let f_grade: f64 = self.calc_f_grade(0.);

        let mut vel_x_dot = (u[0] - f_aero - f_roll - f_grade) / self.mass;
        if x[0] <= 0. && u[0] < f_roll {
            vel_x_dot = 0.;
        }

        na::SVector::<f64, 1>::new(vel_x_dot)
    }

    fn calculate_jacobian(
        &self,
        x: &na::SVector<f64, 1>,
        _u: &na::SVector<f64, 1>,
        _t: f64,
    ) -> (na::SMatrix<f64, 1, 1>, na::SMatrix<f64, 1, 1>) {
        let a = na::SMatrix::<f64, 1, 1>::new(
            -constant::air_density::<f64>()
                * self.area_frontal
                * constant::drag_coeffecient::<f64>()
                * x[0]
                / self.mass,
        );
        let b = na::SMatrix::<f64, 1, 1>::new(-1.0 / self.mass);

        (a, b)
    }

    // TODO: consider negative accelerations whenever the velocity is 0 or approaching 0
    fn calculate_input(
        &self,
        x: &nalgebra::SVector<f64, 1>,
        x_dot_desired: &nalgebra::SVector<f64, 1>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 1> {
        let u_ff = self.mass * x_dot_desired[0]
            + self.calc_f_aero(x[0])
            + self.calc_f_roll(0.)
            + self.calc_f_grade(0.);

        na::SMatrix::<f64, 1, 1>::new(u_ff)
    }

    fn read(&mut self, filename: &str) -> () {
        let data: Model = files::read_toml(filename);

        self.mass = data.mass;
        self.area_frontal = data.area_frontal;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::base::System;
    use approx::assert_relative_eq;

    ///////////////////////////////////////////////////////////////////////////
    /// Test new
    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_new_valid_model() {
        let model = Model::new(1000.0, 2.5);
        assert_eq!(model.mass, 1000.0);
        assert_eq!(model.area_frontal, 2.5);
    }

    #[test]
    #[should_panic(expected = "mass must be positive")]
    fn test_new_negative_mass() {
        Model::new(-1000.0, 2.5);
    }

    #[test]
    #[should_panic(expected = "area_frontal must be positive")]
    fn test_new_negative_area_frontal() {
        Model::new(1000.0, -2.5);
    }

    #[test]
    #[should_panic(expected = "mass must be positive")]
    fn test_new_zero_mass() {
        Model::new(0.0, 2.5);
    }

    #[test]
    #[should_panic(expected = "area_frontal must be positive")]
    fn test_new_zero_area_frontal() {
        Model::new(1000.0, 0.0);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// Test calc_f_aero
    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calc_f_aero_zero_velocity() {
        let model = Model::new(1000.0, 2.0);
        let velocity = 0.0;
        let expected = 0.0;
        assert_relative_eq!(model.calc_f_aero(velocity), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_calc_f_aero_positive_velocity() {
        let model = Model::new(1000.0, 2.0);
        let velocity = 10.0;
        let expected = 0.5
            * constant::air_density::<f64>()
            * model.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity;
        assert_relative_eq!(model.calc_f_aero(velocity), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_calc_f_aero_negative_velocity() {
        let model = Model::new(1000.0, 2.0);
        let velocity = -10.0;
        let expected = 0.5
            * constant::air_density::<f64>()
            * model.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity;
        assert_relative_eq!(model.calc_f_aero(velocity), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_calc_f_aero_large_velocity() {
        let model = Model::new(1000.0, 2.0);
        let velocity = 100.0;
        let expected = 0.5
            * constant::air_density::<f64>()
            * model.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity;
        assert_relative_eq!(model.calc_f_aero(velocity), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_calc_f_aero_small_velocity() {
        let model = Model::new(1000.0, 2.0);
        let velocity = 0.1;
        let expected = 0.5
            * constant::air_density::<f64>()
            * model.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity;
        assert_relative_eq!(model.calc_f_aero(velocity), expected, epsilon = 1e-6);
    }

    ///////////////////////////////////////////////////////////////////////////
    /// Test calc_f_roll
    ///////////////////////////////////////////////////////////////////////////
    #[test]
    fn test_calc_f_roll_zero_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = 0.0;
        let expected_force =
            model.mass * constant::gravity::<f64>() * constant::rolling_resistance::<f64>();
        let actual_force = model.calc_f_roll(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_roll_non_zero_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = std::f64::consts::PI / 4.0;
        let expected_force = model.mass
            * constant::gravity::<f64>()
            * constant::rolling_resistance::<f64>()
            * f64::cos(grade_angle);
        let actual_force = model.calc_f_roll(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_roll_90_degree_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = std::f64::consts::PI / 2.0;
        let expected_force = 0.0;
        let actual_force = model.calc_f_roll(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }

    /////////////////////////////////////////////////////////////////////////////
    /// test calc_f_gravity
    /////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calc_f_grade_positive_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = std::f64::consts::PI / 4.0;
        let expected_force = model.mass * constant::gravity::<f64>() * f64::sin(grade_angle);
        let actual_force = model.calc_f_grade(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_grade_negative_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = -std::f64::consts::PI / 4.0;
        let expected_force = model.mass * constant::gravity::<f64>() * f64::sin(grade_angle);
        let actual_force = model.calc_f_grade(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_grade_zero_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = 0.0;
        let expected_force = 0.0;
        let actual_force = model.calc_f_grade(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_grade_large_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = std::f64::consts::PI / 2.0 - 1e-6;
        let expected_force = model.mass * constant::gravity::<f64>() * f64::sin(grade_angle);
        let actual_force = model.calc_f_grade(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }
    #[test]
    fn test_calc_f_grade_small_grade_angle() {
        let model = Model::new(1000.0, 2.0);
        let grade_angle = 1e-6;
        let expected_force = model.mass * constant::gravity::<f64>() * f64::sin(grade_angle);
        let actual_force = model.calc_f_grade(grade_angle);
        assert_relative_eq!(expected_force, actual_force, epsilon = 1e-6);
    }

    /////////////////////////////////////////////////////////////////////////////
    /// Test get_derivatives
    ////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_get_derivatives_positive_velocity_positive_input() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let u = na::SVector::<f64, 1>::new(100.0);
        let f_aero = model.calc_f_aero(10.0);
        let f_roll = model.calc_f_roll(0.0);
        let f_grade = model.calc_f_grade(0.0);
        let expected = na::SVector::<f64, 1>::new((100.0 - f_aero - f_roll - f_grade) / 1000.0);
        assert_relative_eq!(
            model.get_derivatives(&x, &u, 0.0)[0],
            expected[0],
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_get_derivatives_positive_velocity_negative_input() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let u = na::SVector::<f64, 1>::new(-100.0);
        let f_aero = model.calc_f_aero(10.0);
        let f_roll = model.calc_f_roll(0.0);
        let f_grade = model.calc_f_grade(0.0);
        let expected = na::SVector::<f64, 1>::new((-100.0 - f_aero - f_roll - f_grade) / 1000.0);
        assert_relative_eq!(
            model.get_derivatives(&x, &u, 0.0)[0],
            expected[0],
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_get_derivatives_zero_velocity_positive_input_less_than_rolling_resistance() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(0.0);
        let u = na::SVector::<f64, 1>::new(100.0);
        let expected = na::SVector::<f64, 1>::new(0.0);
        assert_relative_eq!(
            model.get_derivatives(&x, &u, 0.0)[0],
            expected[0],
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_get_derivatives_zero_velocity_negative_input() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(0.0);
        let u = na::SVector::<f64, 1>::new(-100.0);
        let expected = na::SVector::<f64, 1>::new(0.0);
        assert_relative_eq!(
            model.get_derivatives(&x, &u, 0.0)[0],
            expected[0],
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_get_derivatives_velocity_less_than_or_equal_to_zero_input_less_than_rolling_resistance()
    {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(0.0);
        let u = na::SVector::<f64, 1>::new(-50.0);
        let expected = na::SVector::<f64, 1>::new(0.0);
        assert_relative_eq!(
            model.get_derivatives(&x, &u, 0.0)[0],
            expected[0],
            epsilon = 1e-6
        );
    }

    /////////////////////////////////////////////////////////////////////////////
    /// Test calculate_jacobian
    ////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_jacobian_zero_velocity() {
        let model = Model::new(1000.0, 2.5);
        let x = na::SVector::<f64, 1>::new(0.0);
        let (a, b) = model.calculate_jacobian(&x, &na::SVector::<f64, 1>::new(0.0), 0.0);
        assert_relative_eq!(a[(0, 0)], 0.0, epsilon = 1e-6);
        assert_relative_eq!(b[(0, 0)], -1.0 / 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_jacobian_non_zero_velocity() {
        let model = Model::new(1000.0, 2.5);
        let x = na::SVector::<f64, 1>::new(10.0);
        let (a, b) = model.calculate_jacobian(&x, &na::SVector::<f64, 1>::new(0.0), 0.0);
        assert_relative_eq!(
            a[(0, 0)],
            -constant::air_density::<f64>() * 2.5 * constant::drag_coeffecient::<f64>() * 10.0
                / 1000.0,
            epsilon = 1e-6
        );
        assert_relative_eq!(b[(0, 0)], -1.0 / 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_jacobian_different_mass() {
        let model = Model::new(500.0, 2.5);
        let x = na::SVector::<f64, 1>::new(10.0);
        let (a, b) = model.calculate_jacobian(&x, &na::SVector::<f64, 1>::new(0.0), 0.0);
        assert_relative_eq!(
            a[(0, 0)],
            -constant::air_density::<f64>() * 2.5 * constant::drag_coeffecient::<f64>() * 10.0
                / 500.0,
            epsilon = 1e-6
        );
        assert_relative_eq!(b[(0, 0)], -1.0 / 500.0, epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_jacobian_different_frontal_area() {
        let model = Model::new(1000.0, 5.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let (a, b) = model.calculate_jacobian(&x, &na::SVector::<f64, 1>::new(0.0), 0.0);
        assert_relative_eq!(
            a[(0, 0)],
            -constant::air_density::<f64>() * 5.0 * constant::drag_coeffecient::<f64>() * 10.0
                / 1000.0,
            epsilon = 1e-6
        );
        assert_relative_eq!(b[(0, 0)], -1.0 / 1000.0, epsilon = 1e-6);
    }

    /////////////////////////////////////////////////////////////////////////////
    /// Test calculate_input
    ////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_input_positive_x_dot_desired_non_zero_x() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let x_dot_desired = na::SVector::<f64, 1>::new(5.0);
        let expected = na::SVector::<f64, 1>::new(
            5000.0 + model.calc_f_aero(10.0) + model.calc_f_roll(0.0) + model.calc_f_grade(0.0),
        );
        let result = model.calculate_input(&x, &x_dot_desired, 0.0);
        assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_input_zero_x_dot_desired_non_zero_x() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let x_dot_desired = na::SVector::<f64, 1>::new(0.0);
        let expected = na::SVector::<f64, 1>::new(
            model.calc_f_aero(10.0) + model.calc_f_roll(0.0) + model.calc_f_grade(0.0),
        );
        let result = model.calculate_input(&x, &x_dot_desired, 0.0);
        assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_input_negative_x_dot_desired_non_zero_x() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(10.0);
        let x_dot_desired = na::SVector::<f64, 1>::new(-5.0);
        let expected = na::SVector::<f64, 1>::new(
            -5000.0 + model.calc_f_aero(10.0) + model.calc_f_roll(0.0) + model.calc_f_grade(0.0),
        );
        let result = model.calculate_input(&x, &x_dot_desired, 0.0);
        assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_input_positive_x_dot_desired_zero_x() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(0.0);
        let x_dot_desired = na::SVector::<f64, 1>::new(5.0);
        let expected =
            na::SVector::<f64, 1>::new(5000.0 + model.calc_f_roll(0.0) + model.calc_f_grade(0.0));
        let result = model.calculate_input(&x, &x_dot_desired, 0.0);
        assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_input_zero_x_dot_desired_zero_x() {
        let model = Model::new(1000.0, 2.0);
        let x = na::SVector::<f64, 1>::new(0.0);
        let x_dot_desired = na::SVector::<f64, 1>::new(0.0);
        let expected = na::SVector::<f64, 1>::new(model.calc_f_roll(0.0) + model.calc_f_grade(0.0));
        let result = model.calculate_input(&x, &x_dot_desired, 0.0);
        assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    }

    // #[test]
    // fn test_calculate_input_negative_x_dot_desired_zero_x() {
    //     let model = Model::new(1000.0, 2.0);
    //     let x = na::SVector::<f64, 1>::new(0.0);
    //     let x_dot_desired = na::SVector::<f64, 1>::new(-5.0);
    //     let expected =
    //         na::SVector::<f64, 1>::new(-5000.0 + model.calc_f_roll(0.0) + model.calc_f_grade(0.0));
    //     let result = model.calculate_input(&x, &x_dot_desired, 0.0);
    //     assert_relative_eq!(result[0], expected[0], epsilon = 1e-6);
    // }
}
