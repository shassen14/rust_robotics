use nalgebra as na;

/// Generic function pointer alias which represents dx/dt = f(x,t)
pub trait VectorFn<T, const R: usize>: Fn(&na::SVector<T, R>, T) -> na::SVector<T, R> {}

impl<Func: Fn(&na::SVector<T, R>, T) -> na::SVector<T, R>, T, const R: usize> VectorFn<T, R>
    for Func
{
}

/// Type alias hack for integrator to use
pub trait IntegrationFn<T, const R: usize>:
    Fn(&dyn VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>
{
}
impl<
        Func: Fn(&dyn VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>,
        T,
        const R: usize,
    > IntegrationFn<T, R> for Func
{
}

/// Generic function pointer alias which represents dx/dt = f(x,t) using Dynamic Vectors
pub trait VectorFnD<T>: Fn(&na::DVector<T>, T) -> na::DVector<T> {}

/// Type alias hack for integrator to use using Dynamic Vectors
impl<Func: Fn(&na::DVector<T>, T) -> na::DVector<T>, T> VectorFnD<T> for Func {}

pub trait IntegrationFnD<T>:
    Fn(&dyn VectorFnD<T>, &na::DVector<T>, T, T) -> na::DVector<T>
{
}
impl<Func: Fn(&dyn VectorFnD<T>, &na::DVector<T>, T, T) -> na::DVector<T>, T> IntegrationFnD<T>
    for Func
{
}

// Old way but this did not take in any outside variables
// Praying for trait alias or type alias for impl
// pub type VectorFn<T, const R: usize> = fn(&na::SVector<T, R>, T) -> na::SVector<T, R>;

// Old way but this did not take in any outside variables
// Praying for trait alias or type alias for impl
// pub type IntegrationFn<T, const R: usize> =
//     fn(impl VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>;
