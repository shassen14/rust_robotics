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
    pub fn new(mass: f64, area_frontal: f64) -> Self {
        Model {
            mass: mass,
            area_frontal: area_frontal,
        }
    }

    fn calc_f_aero(&self, velocity: f64) -> f64 {
        0.5 * constant::air_density::<f64>()
            * self.area_frontal
            * constant::drag_coeffecient::<f64>()
            * velocity
            * velocity
    }

    // TODO: assume radians
    fn calc_f_roll(&self, grade_angle: f64) -> f64 {
        self.mass
            * constant::gravity::<f64>()
            * constant::rolling_resistance::<f64>()
            * f64::cos(grade_angle)
    }

    fn calc_f_grade(&self, grade_angle: f64) -> f64 {
        self.mass * constant::gravity::<f64>() * f64::sin(grade_angle)
    }
}

/// Bicycle Kinematic Model with 3 states and 2 inputs
///
/// x_dot = [vel_x_dot], vehicle frame
/// x = [vel_x]
/// u = [F], F_engine/tractive > 0 | F_brake < 0
/// TODO: technically we could go reverse if f_brake is greater than everything else
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
        // TODO: include grade road angle
        let f_aero: f64 = self.calc_f_aero(x[0]);
        let f_roll: f64 = self.calc_f_roll(0.);
        let f_grade: f64 = self.calc_f_grade(0.);
        let vel_x_dot: f64 = (u[0] - f_aero - f_roll - f_grade) / self.mass;

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

    fn calculate_input(
        &self,
        x: &nalgebra::SVector<f64, 1>,
        x_dot_desired: &nalgebra::SVector<f64, 1>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 1> {
        let f_aero: f64 = self.calc_f_aero(x[0]);
        let f_roll: f64 = self.calc_f_roll(0.);
        let f_grade: f64 = self.calc_f_grade(0.);

        let u_ff: f64 = self.mass * x_dot_desired[0] + f_aero + f_roll + f_grade;

        na::SMatrix::<f64, 1, 1>::new(u_ff)
    }

    fn read(&mut self, filename: &str) -> () {
        let data: Model = files::read_toml(filename);

        self.mass = data.mass;
        self.area_frontal = data.area_frontal;
    }
}
