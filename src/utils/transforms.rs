// rust_robotics
use crate::utils::convert;
use crate::utils::defs::AngleUnits;

// 3rd Party
use nalgebra::{self as na};

/// x, y, z, roll, pitch, yaw
/// TODO:
#[allow(unused)]
pub struct FrameTransform3<T> {
    b_to_i_iso: na::Isometry3<T>,
    i_to_b_iso: na::Isometry3<T>,
}

impl<T> FrameTransform3<T>
where
    T: na::RealField + num_traits::Float + Copy,
{
    pub fn new(offsets: &[T; 6], unit: Option<AngleUnits>) -> Self {
        let mut roll = offsets[3];
        let mut pitch = offsets[4];
        let mut yaw = offsets[5];

        // If unit is radian, then do nothing.
        // If unit is None or degree, convert to radian. Default API
        match unit {
            Some(AngleUnits::Radian) => {}
            _ => {
                roll = convert::deg_to_rad(roll);
                pitch = convert::deg_to_rad(pitch);
                yaw = convert::deg_to_rad(yaw);
            }
        }

        let b_to_i_q = na::UnitQuaternion::<T>::from_euler_angles(roll, pitch, yaw);
        let b_to_i_t = na::Translation3::<T>::new(offsets[0], offsets[1], offsets[2]);
        let b_to_i_i = na::Isometry3::from_parts(b_to_i_t, b_to_i_q);
        let i_to_b_i = b_to_i_i.inverse();

        FrameTransform3 {
            b_to_i_iso: b_to_i_i,
            i_to_b_iso: i_to_b_i,
        }
    }

    pub fn get_b_to_i_iso(&self) -> na::Isometry3<T> {
        self.b_to_i_iso
    }

    pub fn get_i_to_b_iso(&self) -> na::Isometry3<T> {
        self.i_to_b_iso
    }

    pub fn angle_b_to_i(&self, angle_body: &na::Point3<T>) -> na::Point3<T> {
        self.b_to_i_iso.rotation * angle_body
    }

    pub fn angle_i_to_b(&self, angle_inertial: &na::Point3<T>) -> na::Point3<T> {
        self.i_to_b_iso.rotation * angle_inertial
    }

    pub fn angular_velocity_b_to_i(&self, angular_velocity_body: &na::Point3<T>) -> na::Point3<T> {
        self.b_to_i_iso.rotation * angular_velocity_body
    }

    pub fn angular_velocity_i_to_b(
        &self,
        angular_velocity_inertial: &na::Point3<T>,
    ) -> na::Point3<T> {
        self.i_to_b_iso.rotation * angular_velocity_inertial
    }

    pub fn angular_acceleration_b_to_i(
        &self,
        angular_acceleration_body: &na::Point3<T>,
    ) -> na::Point3<T> {
        self.b_to_i_iso.rotation * angular_acceleration_body
    }

    pub fn angular_acceleration_i_to_b(
        &self,
        angular_acceleration_inertial: &na::Point3<T>,
    ) -> na::Point3<T> {
        self.i_to_b_iso.rotation * angular_acceleration_inertial
    }

    pub fn point_b_to_i(&self, point: &na::Point3<T>) -> na::Point3<T> {
        self.b_to_i_iso.transform_point(point)
    }

    pub fn point_i_to_b(&self, point: &na::Point3<T>) -> na::Point3<T> {
        self.i_to_b_iso.transform_point(point)
    }
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use core::f64;

    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    const ROTATION: [f64; 6] = [0., 0., 0., 90., 90., 90.];
    const TRANSLATION: [f64; 6] = [1., 2., 3., 0., 0., 0.];
    const COMBINED: [f64; 6] = [1., 2., 3., 90., 90., 90.];

    #[test]
    fn test_transform_pt_rotation() {
        let tf = FrameTransform3::<f64>::new(&ROTATION, Some(AngleUnits::Degree));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let point = Point3::<f64>::new(1., 2., 3.);

        // equivalent to tf.get_b_to_i_iso().rotation * point
        let tf_point = tf.point_b_to_i(&point);
        let answer_tf_point = Point3::new(3.0, 2.0, -1.0);

        let tf_point2 = tf.point_i_to_b(&point);
        let answer_tf_point2 = Point3::new(-3.0, 2.0, 1.0);

        let tf_tf_point = tf.point_i_to_b(&tf_point);

        println!("tf_point: {}", tf_point);
        println!("answer_tf_point: {}", answer_tf_point);
        for i in 0..2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..2 {
            assert_relative_eq!(tf_tf_point[i], point[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_pt_translation() {
        let tf = FrameTransform3::<f64>::new(&TRANSLATION, Some(AngleUnits::Degree));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let point = Point3::<f64>::new(1., 2., 3.);

        // equivalent to tf.get_b_to_i_iso().translation * point
        let tf_point = tf.point_b_to_i(&point);
        let answer_tf_point = Point3::new(2.0, 4.0, 6.0);

        let tf_point2 = tf.point_i_to_b(&point);
        let answer_tf_point2 = Point3::new(0.0, 0.0, 0.0);

        let tf_tf_point = tf.point_i_to_b(&tf_point);

        println!("tf_point: {}", tf_point);
        println!("answer_tf_point: {}", answer_tf_point);
        for i in 0..2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..2 {
            assert_relative_eq!(tf_tf_point[i], point[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_pt_combined() {
        let tf = FrameTransform3::<f64>::new(&COMBINED, None);

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let point = Point3::<f64>::new(1., 2., 3.);

        // equivalent tf.get_b_to_i_iso().translation * (tf.get_b_to_i_iso().rotation * point)
        let tf_point = tf.point_b_to_i(&point);
        let answer_tf_point = Point3::new(4.0, 4.0, 2.0);

        let tf_point2 = tf.point_i_to_b(&point);
        let answer_tf_point2 = Point3::new(0.0, 0.0, 0.0);

        let tf_tf_point = tf.point_i_to_b(&tf_point);

        println!("tf_point: {}", tf_point);
        println!("answer_tf_point: {}", answer_tf_point);
        for i in 0..2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..2 {
            assert_relative_eq!(tf_tf_point[i], point[i], epsilon = 1e-12);
        }
    }
}
