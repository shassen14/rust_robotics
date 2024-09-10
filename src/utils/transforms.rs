// rust_robotics
use crate::utils::convert;
use crate::utils::defs::AngleUnits;

// 3rd Party
use nalgebra::{self as na};

/// x, y, z, roll, pitch, yaw
/// TODO: should i switch the outputs to be points or vectors?
#[allow(unused)]
pub struct FrameTransform3<T> {
    b_to_i_iso: na::Isometry3<T>,
    i_to_b_iso: na::Isometry3<T>,
    angle_unit: AngleUnits,
}

impl<T> FrameTransform3<T>
where
    T: na::RealField
        + num_traits::Float
        + Copy
        + std::ops::Mul<na::SVector<T, 3>, Output = na::SVector<T, 3>>,
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
        };

        let b_to_i_q = na::UnitQuaternion::<T>::from_euler_angles(roll, pitch, yaw);
        let b_to_i_t = na::Translation3::<T>::new(offsets[0], offsets[1], offsets[2]);
        let b_to_i_i = na::Isometry3::from_parts(b_to_i_t, b_to_i_q);
        let i_to_b_i = b_to_i_i.inverse();

        FrameTransform3 {
            b_to_i_iso: b_to_i_i,
            i_to_b_iso: i_to_b_i,
            angle_unit: unit.unwrap_or(AngleUnits::Degree),
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

    pub fn linear_velocity_b_to_i(
        &self,
        linear_velocity_body: &na::Point3<T>,
        angular_velocity_body: &na::Point3<T>,
    ) -> na::Point3<T> {
        // Should be okay to unwrap.
        // Convert units to rad/s
        let ang_vel_i = self
            .angular_velocity_b_to_i(angular_velocity_body)
            .to_homogeneous();
        let angular_velocity_inertial_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_vel_i[0], ang_vel_i[1], ang_vel_i[2]);

        // Calculate the angular velocity contribution due to translation lever arm
        // Add together because of superposition
        self.b_to_i_iso.rotation * linear_velocity_body
            + angular_velocity_inertial_vec.cross(&self.b_to_i_iso.translation.vector)
    }

    pub fn linear_velocity_i_to_b(
        &self,
        linear_velocity_inertial: &na::Point3<T>,
        angular_velocity_inertial: &na::Point3<T>,
    ) -> na::Point3<T> {
        // Should be okay to unwrap.
        // Convert units to rad/s
        let ang_vel_b = self
            .angular_velocity_i_to_b(angular_velocity_inertial)
            .to_homogeneous();
        let angular_velocity_body_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_vel_b[0], ang_vel_b[1], ang_vel_b[2]);

        // Calculate the angular velocity contribution due to translation lever arm
        // Add together because of superposition
        self.i_to_b_iso.rotation * linear_velocity_inertial
            + angular_velocity_body_vec.cross(&self.i_to_b_iso.translation.vector)
    }

    pub fn linear_acceleration_b_to_i(
        &self,
        linear_acceleration_body: &na::Point3<T>,
        angular_acceleration_body: &na::Point3<T>,
        angular_velocity_body: &na::Point3<T>,
    ) -> na::Point3<T> {
        // Should be okay to unwrap.
        // Convert units to rad/s
        let ang_accel_i = self
            .angular_acceleration_b_to_i(angular_acceleration_body)
            .to_homogeneous();
        let angular_acceleration_inertial_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_accel_i[0], ang_accel_i[1], ang_accel_i[2]);

        let ang_vel_i = self
            .angular_velocity_b_to_i(angular_velocity_body)
            .to_homogeneous();
        let angular_velocity_inertial_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_vel_i[0], ang_vel_i[1], ang_vel_i[2]);

        // Rotate linear body accelerations to align with inertial frame.
        // Calculate angular accelerations' contributions by taking its cross product with
        // the lever arm (translations).
        // Calculate the centripetal acceleration using the angular velocity cross product twice
        // with the lever arm (translations).
        // Add together because of superposition
        self.b_to_i_iso.rotation * linear_acceleration_body
            + angular_acceleration_inertial_vec.cross(&self.b_to_i_iso.translation.vector)
            + angular_velocity_inertial_vec
                .cross(&angular_velocity_inertial_vec.cross(&self.b_to_i_iso.translation.vector))
    }

    pub fn linear_acceleration_i_to_b(
        &self,
        linear_acceleration_inertial: &na::Point3<T>,
        angular_acceleration_inertial: &na::Point3<T>,
        angular_velocity_inertial: &na::Point3<T>,
    ) -> na::Point3<T> {
        // Should be okay to unwrap.
        // Convert units to rad/s
        let ang_accel_b = self
            .angular_acceleration_i_to_b(angular_acceleration_inertial)
            .to_homogeneous();
        let angular_acceleration_inertial_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_accel_b[0], ang_accel_b[1], ang_accel_b[2]);

        let ang_vel_b = self
            .angular_velocity_i_to_b(angular_velocity_inertial)
            .to_homogeneous();
        let angular_velocity_body_vec = self.angle_conversion_factor()
            * na::Vector3::new(ang_vel_b[0], ang_vel_b[1], ang_vel_b[2]);

        // Rotate linear body accelerations to align with inertial frame.
        // Calculate angular accelerations' contributions by taking its cross product with
        // the lever arm (translations).
        // Calculate the centripetal acceleration using the angular velocity cross product twice
        // with the lever arm (translations).
        // Add together because of superposition
        self.i_to_b_iso.rotation * linear_acceleration_inertial
            + angular_acceleration_inertial_vec.cross(&self.i_to_b_iso.translation.vector)
            + angular_velocity_body_vec
                .cross(&angular_velocity_body_vec.cross(&self.i_to_b_iso.translation.vector))
    }

    fn angle_conversion_factor(&self) -> T {
        match self.angle_unit {
            AngleUnits::Radian => T::from(1).unwrap(),
            AngleUnits::Degree => convert::deg_to_rad(T::from(1).unwrap()),
        }
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
        for i in 0..=2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..=2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..=2 {
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
        for i in 0..=2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..=2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..=2 {
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
        for i in 0..=2 {
            assert_relative_eq!(tf_point[i], answer_tf_point[i], epsilon = 1e-12);
        }

        println!("tf_point2: {}", tf_point2);
        println!("answer_tf_point2: {}", answer_tf_point2);
        for i in 0..=2 {
            assert_relative_eq!(tf_point2[i], answer_tf_point2[i], epsilon = 1e-12);
        }

        println!("tf_tf_point: {}", tf_tf_point);
        println!("point: {}", point);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_point[i], point[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_angle_rotation() {
        let tf = FrameTransform3::<f64>::new(&ROTATION, None);

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let angle = Point3::<f64>::new(90., 0., 90.);

        // equivalent to tf.get_b_to_i_iso().rotation * angle
        let tf_angle = tf.angle_b_to_i(&angle);
        let answer_tf_angle = Point3::new(90., 0., -90.);

        let tf_angle2 = tf.angle_i_to_b(&angle);
        let answer_tf_angle2 = Point3::new(-90., 0.0, 90.);

        let tf_tf_angle = tf.angle_i_to_b(&tf_angle);

        println!("tf_angle: {}", tf_angle);
        println!("answer_tf_angle: {}", answer_tf_angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle[i], answer_tf_angle[i], epsilon = 1e-12);
        }

        println!("tf_angle2: {}", tf_angle2);
        println!("answer_tf_angle2: {}", answer_tf_angle2);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle2[i], answer_tf_angle2[i], epsilon = 1e-12);
        }

        println!("tf_tf_angle: {}", tf_tf_angle);
        println!("angle: {}", angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_angle[i], angle[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_angle_translation() {
        let tf = FrameTransform3::<f64>::new(&TRANSLATION, Some(AngleUnits::Radian));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let angle =
            Point3::<f64>::new(std::f64::consts::FRAC_PI_2, 0., std::f64::consts::FRAC_PI_2);

        // equivalent to tf.get_b_to_i_iso().rotation * angle
        let tf_angle = tf.angle_b_to_i(&angle);
        let answer_tf_angle = angle;

        let tf_angle2 = tf.angle_i_to_b(&angle);
        let answer_tf_angle2 = angle;

        let tf_tf_angle = tf.angle_i_to_b(&tf_angle);

        println!("tf_angle: {}", tf_angle);
        println!("answer_tf_angle: {}", answer_tf_angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle[i], answer_tf_angle[i], epsilon = 1e-12);
        }

        println!("tf_angle2: {}", tf_angle2);
        println!("answer_tf_angle2: {}", answer_tf_angle2);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle2[i], answer_tf_angle2[i], epsilon = 1e-12);
        }

        println!("tf_tf_angle: {}", tf_tf_angle);
        println!("angle: {}", angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_angle[i], angle[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_ang_vel_rotation() {
        let tf = FrameTransform3::<f64>::new(&ROTATION, None);

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let angle = Point3::<f64>::new(90., 0., 90.);

        // equivalent to tf.get_b_to_i_iso().rotation * angle
        let tf_angle = tf.angular_velocity_b_to_i(&angle);
        let answer_tf_angle = Point3::new(90., 0., -90.);

        let tf_angle2 = tf.angular_velocity_i_to_b(&angle);
        let answer_tf_angle2 = Point3::new(-90., 0.0, 90.);

        let tf_tf_angle = tf.angular_velocity_i_to_b(&tf_angle);

        println!("tf_angle: {}", tf_angle);
        println!("answer_tf_angle: {}", answer_tf_angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle[i], answer_tf_angle[i], epsilon = 1e-12);
        }

        println!("tf_angle2: {}", tf_angle2);
        println!("answer_tf_angle2: {}", answer_tf_angle2);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle2[i], answer_tf_angle2[i], epsilon = 1e-12);
        }

        println!("tf_tf_angle: {}", tf_tf_angle);
        println!("angle: {}", angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_angle[i], angle[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_ang_acc_rotation() {
        let tf = FrameTransform3::<f64>::new(&ROTATION, None);

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let angle = Point3::<f64>::new(90., 0., 90.);

        // equivalent to tf.get_b_to_i_iso().rotation * angle
        let tf_angle = tf.angular_acceleration_b_to_i(&angle);
        let answer_tf_angle = Point3::new(90., 0., -90.);

        let tf_angle2 = tf.angular_acceleration_i_to_b(&angle);
        let answer_tf_angle2 = Point3::new(-90., 0.0, 90.);

        let tf_tf_angle = tf.angular_acceleration_i_to_b(&tf_angle);

        println!("tf_angle: {}", tf_angle);
        println!("answer_tf_angle: {}", answer_tf_angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle[i], answer_tf_angle[i], epsilon = 1e-12);
        }

        println!("tf_angle2: {}", tf_angle2);
        println!("answer_tf_angle2: {}", answer_tf_angle2);
        for i in 0..=2 {
            assert_relative_eq!(tf_angle2[i], answer_tf_angle2[i], epsilon = 1e-12);
        }

        println!("tf_tf_angle: {}", tf_tf_angle);
        println!("angle: {}", angle);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_angle[i], angle[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_lin_vel_rotation() {
        let tf = FrameTransform3::<f64>::new(&ROTATION, Some(AngleUnits::Degree));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let velocity = Point3::<f64>::new(1., 2., 3.);
        let angular_velocity_body = Point3::<f64>::new(3., 2., 1.);
        let angular_velocity_inertial = tf.angular_velocity_b_to_i(&angular_velocity_body);

        // equivalent to tf.get_b_to_i_iso().rotation * point
        let tf_velocity = tf.linear_velocity_b_to_i(&velocity, &angular_velocity_body);
        let answer_tf_velocity = Point3::new(3.0, 2.0, -1.0);

        let tf_velocity2 = tf.linear_velocity_i_to_b(&velocity, &angular_velocity_inertial);
        let answer_tf_velocity2 = Point3::new(-3.0, 2.0, 1.0);

        let tf_tf_velocity = tf.linear_velocity_i_to_b(&tf_velocity, &angular_velocity_inertial);

        println!("tf_velocity: {}", tf_velocity);
        println!("answer_tf_velocity: {}", answer_tf_velocity);
        for i in 0..=2 {
            assert_relative_eq!(tf_velocity[i], answer_tf_velocity[i], epsilon = 1e-12);
        }

        println!("tf_velocity2: {}", tf_velocity2);
        println!("answer_tf_velocity2: {}", answer_tf_velocity2);
        for i in 0..=2 {
            assert_relative_eq!(tf_velocity2[i], answer_tf_velocity2[i], epsilon = 1e-12);
        }

        println!("tf_tf_velocity: {}", tf_tf_velocity);
        println!("velocity: {}", velocity);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_velocity[i], velocity[i], epsilon = 1e-12);
        }
    }

    #[test]
    fn test_transform_lin_vel_translation() {
        let tf = FrameTransform3::<f64>::new(&TRANSLATION, Some(AngleUnits::Radian));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let velocity = Point3::<f64>::new(1., 2., 3.);
        let angular_velocity_body = Point3::<f64>::new(
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
        );
        let angular_velocity_inertial = tf.angular_velocity_b_to_i(&angular_velocity_body);

        // equivalent to tf.get_b_to_i_iso().rotation * point
        let tf_velocity = tf.linear_velocity_b_to_i(&velocity, &angular_velocity_body);
        // answer has to do with the original linear velocity and the angular velocity cross product with translations
        let answer_tf_velocity = Point3::new(
            1.0 + std::f64::consts::FRAC_PI_2 * 1.0,
            2.0 + std::f64::consts::FRAC_PI_2 * -2.0,
            3.0 + std::f64::consts::FRAC_PI_2 * 1.0,
        );

        let tf_velocity2 = tf.linear_velocity_i_to_b(&velocity, &angular_velocity_inertial);
        // answer has to do with the original linear velocity and the angular velocity cross product with translations
        let answer_tf_velocity2 = Point3::new(
            1.0 + std::f64::consts::FRAC_PI_2 * -1.0,
            2.0 + std::f64::consts::FRAC_PI_2 * 2.0,
            3.0 + std::f64::consts::FRAC_PI_2 * -1.0,
        );

        // should be the original velocity
        let tf_tf_velocity = tf.linear_velocity_i_to_b(&tf_velocity, &angular_velocity_inertial);

        println!("tf_velocity: {}", tf_velocity);
        println!("answer_tf_velocity: {}", answer_tf_velocity);
        for i in 0..=2 {
            assert_relative_eq!(tf_velocity[i], answer_tf_velocity[i], epsilon = 1e-12);
        }

        println!("tf_velocity2: {}", tf_velocity2);
        println!("answer_tf_velocity2: {}", answer_tf_velocity2);
        for i in 0..=2 {
            assert_relative_eq!(tf_velocity2[i], answer_tf_velocity2[i], epsilon = 1e-12);
        }

        println!("tf_tf_velocity: {}", tf_tf_velocity);
        println!("velocity: {}", velocity);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_velocity[i], velocity[i], epsilon = 1e-12);
        }
    }

    // TODO: Only tests if acceleration transforms forward and back between frames. Think of simple cases of knowing the
    // answer intuitively
    #[test]
    fn test_transform_lin_acc_combined() {
        let tf = FrameTransform3::<f64>::new(&COMBINED, Some(AngleUnits::Radian));

        println!("b to i Isometry: {}", tf.get_b_to_i_iso());
        println!("i to b Isometry: {}", tf.get_i_to_b_iso());

        let acceleration = Point3::<f64>::new(1., 2., 3.);
        let angular_acceleration_body = Point3::<f64>::new(
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
        );
        let angular_velocity_body = Point3::<f64>::new(
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::FRAC_PI_2,
        );
        let angular_acceleration_inertial =
            tf.angular_acceleration_b_to_i(&angular_acceleration_body);
        let angular_velocity_inertial = tf.angular_velocity_b_to_i(&angular_velocity_body);

        // equivalent to tf.get_b_to_i_iso().rotation * point
        let tf_acceleration = tf.linear_acceleration_b_to_i(
            &acceleration,
            &angular_acceleration_body,
            &angular_velocity_body,
        );
        // answer has to do with the original linear acceleration and the angular acceleration cross product with translations
        // let answer_tf_acceleration = Point3::new(
        //     1.0 + std::f64::consts::FRAC_PI_2 * 1.0,
        //     2.0 + std::f64::consts::FRAC_PI_2 * -2.0,
        //     3.0 + std::f64::consts::FRAC_PI_2 * 1.0,
        // );

        // let tf_acceleration2 = tf.linear_acceleration_i_to_b(
        //     &acceleration,
        //     &angular_acceleration_inertial,
        //     &angular_velocity_inertial,
        // );
        // // answer has to do with the original linear acceleration and the angular acceleration cross product with translations
        // let answer_tf_acceleration2 = Point3::new(
        //     1.0 + std::f64::consts::FRAC_PI_2 * -1.0,
        //     2.0 + std::f64::consts::FRAC_PI_2 * 2.0,
        //     3.0 + std::f64::consts::FRAC_PI_2 * -1.0,
        // );

        // should be the original acceleration
        let tf_tf_acceleration = tf.linear_acceleration_i_to_b(
            &tf_acceleration,
            &angular_acceleration_inertial,
            &angular_velocity_inertial,
        );

        // println!("tf_acceleration: {}", tf_acceleration);
        // println!("answer_tf_acceleration: {}", answer_tf_acceleration);
        // for i in 0..=2 {
        //     assert_relative_eq!(
        //         tf_acceleration[i],
        //         answer_tf_acceleration[i],
        //         epsilon = 1e-12
        //     );
        // }

        // println!("tf_acceleration2: {}", tf_acceleration2);
        // println!("answer_tf_acceleration2: {}", answer_tf_acceleration2);
        // for i in 0..=2 {
        //     assert_relative_eq!(
        //         tf_acceleration2[i],
        //         answer_tf_acceleration2[i],
        //         epsilon = 1e-12
        //     );
        // }

        println!("tf_tf_acceleration: {}", tf_tf_acceleration);
        println!("acceleration: {}", acceleration);
        for i in 0..=2 {
            assert_relative_eq!(tf_tf_acceleration[i], acceleration[i], epsilon = 1e-12);
        }
    }
}
