use rust_robotics::num_methods::runge_kutta;

use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra as na;

fn sin(_: &na::SVector<f64, 1>, t: f64) -> na::SVector<f64, 1> {
    na::SVector::<f64, 1>::new(f64::sin(t))
}

fn sin_d(_: &na::DVector<f64>, t: f64) -> na::DVector<f64> {
    na::dvector![f64::sin(t)]
}

fn ca(x: &na::SVector<f64, 3>, _: f64) -> na::SVector<f64, 3> {
    let a: na::SMatrix<f64, 3, 3> =
        na::SMatrix::<f64, 3, 3>::new(0., 1., 0., 0., 0., 1., 0., 0., 0.);
    a * x
}

fn ca_d(x: &na::DVector<f64>, _: f64) -> na::DVector<f64> {
    let a: na::DMatrix<f64> = na::dmatrix![0., 1., 0.; 0., 0., 1.; 0., 0., 0.];
    a * x
}

fn rk1_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 1>::new(0.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk1_one_timestep", |b| {
        b.iter(|| runge_kutta::rk1(&sin, &x0, t0, tf))
    });
}

fn rk2_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 1>::new(0.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk2_one_timestep", |b| {
        b.iter(|| runge_kutta::rk2(&sin, &x0, t0, tf))
    });
}

fn rk3_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 1>::new(0.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk3_one_timestep", |b| {
        b.iter(|| runge_kutta::rk3(&sin, &x0, t0, tf))
    });
}

fn rk4_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 1>::new(0.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk4_one_timestep", |b| {
        b.iter(|| runge_kutta::rk4(&sin, &x0, t0, tf))
    });
}

fn rk1d_benchmark(c: &mut Criterion) {
    let x0 = na::DVector::<f64>::zeros(1);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk1d_one_timestep", |b| {
        b.iter(|| runge_kutta::rk1d(&sin_d, &x0, t0, tf))
    });
}

fn rk2d_benchmark(c: &mut Criterion) {
    let x0 = na::DVector::<f64>::zeros(1);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk2d_one_timestep", |b| {
        b.iter(|| runge_kutta::rk2d(&sin_d, &x0, t0, tf))
    });
}

fn rk3d_benchmark(c: &mut Criterion) {
    let x0 = na::DVector::<f64>::zeros(1);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk3d_one_timestep", |b| {
        b.iter(|| runge_kutta::rk3d(&sin_d, &x0, t0, tf))
    });
}

fn rk4d_benchmark(c: &mut Criterion) {
    let x0 = na::DVector::<f64>::zeros(1);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk4d_one_timestep", |b| {
        b.iter(|| runge_kutta::rk4d(&sin_d, &x0, t0, tf))
    });
}

///////////////////////////////////////////////////////////////////////////////
// Constant Acceleration
///////////////////////////////////////////////////////////////////////////////

fn rk1_ca_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 3>::new(0., 1., 1.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk1_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk1(&ca, &x0, t0, tf))
    });
}

fn rk2_ca_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 3>::new(0., 1., 1.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk2_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk2(&ca, &x0, t0, tf))
    });
}

fn rk3_ca_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 3>::new(0., 1., 1.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk3_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk3(&ca, &x0, t0, tf))
    });
}

fn rk4_ca_benchmark(c: &mut Criterion) {
    let x0 = na::SVector::<f64, 3>::new(0., 1., 1.);
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk4_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk4(&ca, &x0, t0, tf))
    });
}

fn rk1d_ca_benchmark(c: &mut Criterion) {
    let x0 = na::dvector![0., 1., 1.];
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk1d_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk1d(&ca_d, &x0, t0, tf))
    });
}

fn rk2d_ca_benchmark(c: &mut Criterion) {
    let x0 = na::dvector![0., 1., 1.];
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk2d_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk2d(&ca_d, &x0, t0, tf))
    });
}

fn rk3d_ca_benchmark(c: &mut Criterion) {
    let x0 = na::dvector![0., 1., 1.];
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk3d_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk3d(&ca_d, &x0, t0, tf))
    });
}

fn rk4d_ca_benchmark(c: &mut Criterion) {
    let x0 = na::dvector![0., 1., 1.];
    let t0 = 0.0;
    let tf = 0.1;
    c.bench_function("rk4d_ca_one_timestep", |b| {
        b.iter(|| runge_kutta::rk4d(&ca_d, &x0, t0, tf))
    });
}

criterion_group!(
    benches,
    rk1_benchmark,
    rk2_benchmark,
    rk3_benchmark,
    rk4_benchmark,
    rk1d_benchmark,
    rk2d_benchmark,
    rk3d_benchmark,
    rk4d_benchmark,
    rk1_ca_benchmark,
    rk2_ca_benchmark,
    rk3_ca_benchmark,
    rk4_ca_benchmark,
    rk1d_ca_benchmark,
    rk2d_ca_benchmark,
    rk3d_ca_benchmark,
    rk4d_ca_benchmark,
);
criterion_main!(benches);
