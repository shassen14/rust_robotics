use num_traits::Float;

/// Gravitational acceleration [m/s^2]
pub fn gravity<T: Float>() -> T {
    T::from(9.81).unwrap()
}

/// Air density [kg/m^3]
pub fn air_density<T: Float>() -> T {
    T::from(1.225).unwrap()
}

/// Drag coefficient
pub fn drag_coeffecient<T: Float>() -> T {
    T::from(0.3).unwrap()
}

/// Rolling resistance coefficient
pub fn rolling_resistance<T: Float>() -> T {
    T::from(0.015).unwrap()
}
