extern crate nalgebra as na;

use crate::models::base;

#[derive(Clone, Copy)]
pub struct Model {
    length_front: f64,
    length_rear: f64,
}

impl Model {
    fn new(lf: f64, lr: f64) -> Self {
        Model {
            length_front: lf,
            length_rear: lr,
        }
    }

    fn calculate_length_total(self) -> f64 {
        self.length_front + self.length_rear
    }

    fn calculate_sideslip(self, steer_angle: f64) -> f64 {
        f64::atan(self.length_rear * f64::tan(steer_angle) / self.calculate_length_total())
    }

    fn calculate_f(self, steer_angle: f64) -> na::SMatrix<f64, 3, 3> {
        na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
        ]));
        // let x1
        todo!();
    }
}
// TODO: might add two more states being global x, and global y
/// x_dot = [pos_x_dot, pos_y_dot, heading_dot], body frame
/// u = [vel_x, steering_angle], body frame
impl base::System<f64, 3, 2> for Model {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 3>,
        u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> na::SVector<f64, 3> {
        let sideslip: f64 = self.calculate_sideslip(u[1]);
        let pos_x_dot: f64 = u[0] * f64::cos(x[2] + sideslip);
        let pos_y_dot: f64 = u[0] * f64::cos(x[2] + sideslip);
        let yaw_dot: f64 = f64::tan(u[1]) * f64::cos(sideslip) / self.calculate_length_total();
        na::SVector::<f64, 3>::new(pos_x_dot, pos_y_dot, yaw_dot)
    }

    fn get_jacobian(
        &self,
        _x: &nalgebra::SVector<f64, 3>,
        _u: &nalgebra::SVector<f64, 2>,
        _t: f64,
    ) -> (na::SMatrix<f64, 3, 3>, na::SMatrix<f64, 2, 2>) {
        (
            Model::calculate_f(Model::new(self.length_front, self.length_rear), 5.),
            na::SMatrix::<f64, 2, 2>::zeros(),
        );
        todo!();
    }
}
