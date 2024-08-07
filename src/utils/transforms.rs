use nalgebra::{self as na};

/////////////////////////////////////////////////////////////////////////

#[allow(unused)]
/// Frame transformation on input position from body to inertial frame
///
/// # Arguments
///
/// * `point` - Position in body frame [x, y, z]
/// * Returns inertial position after frame transform
///
pub fn point_b_to_i<T>(point: &na::SVector<T, 3>, tf: &na::Affine3<T>) -> na::SVector<T, 3>
where
    T: na::RealField,
{
    todo!();
}

/////////////////////////////////////////////////////////////////////////

#[allow(unused)]
/// Frame transformation on input position from inertial to body frame
///
/// # Arguments
///
/// * `point` - Position in inertial frame [x, y, z]
/// * Returns body position after frame transform
///
pub fn point_i_to_b<T>(point: &na::SVector<T, 3>, tf: &na::Affine3<T>) -> na::SVector<T, 3>
where
    T: na::RealField,
{
    todo!();
}
