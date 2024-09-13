use rust_robotics::num_methods::runge_kutta;

use codspeed_criterion_compat::{criterion_group, criterion_main, Criterion};
use nalgebra as na;

fn sin(_: &na::SVector<f64, 1>, t: f64) -> na::SVector<f64, 1> {
    na::SVector::<f64, 1>::new(f64::sin(t))
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
criterion_group!(
    benches,
    rk1_benchmark,
    rk2_benchmark,
    rk3_benchmark,
    rk4_benchmark
);
criterion_main!(benches);
