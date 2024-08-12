// rust robotics
// use rust_robotics::models::base;
use rust_robotics::models::base::System;
use rust_robotics::models::bicycle_kinematic;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::defs;
use rust_robotics::utils::plot;

// 3rd party or std
use nalgebra as na;
use plotters::prelude::*;
// use plotters::{chart::ChartState, coord::types::RangedCoordf64};
use core::f64;
use minifb::{Key, KeyRepeat};
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::error::Error;
use std::time::SystemTime;

// TODO: read from config yml or some type of file for runtime instead of compile time
const WIDTH: usize = 1280;
const HEIGHT: usize = 720;
const SAMPLE_RATE: f64 = 100f64;
const FPS: f64 = 30f64;
// const STATES: usize = 3;
// const INPUTS: usize = 2;

fn main() -> Result<(), Box<dyn Error>> {
    let mut buf = defs::BufferWrapper(vec![0u32; WIDTH * HEIGHT]);

    let mut window = plot::create_window("Kinematic Bicycle Model Positioning", WIDTH, HEIGHT);

    // TODO: magic numbers
    let cs = plot::create_2d_chartstate(
        buf.borrow_mut(),
        WIDTH as u32,
        HEIGHT as u32,
        10,
        30,
        "sans-serif",
        15,
        &plotters::style::RGBColor(0u8, 255u8, 0u8),
        [-100.0, 100.0],
        [-100.0, 100.0],
    );

    // let cs = {
    //     // create an image using a buffer of integers that represent the colo
    //     let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
    //         buf.borrow_mut(),
    //         (WIDTH as u32, HEIGHT as u32),
    //     )?
    //     .into_drawing_area();
    //     root.fill(&BLACK)?;

    //     // create 2d plane here
    //     let mut chart = ChartBuilder::on(&root)
    //         .margin(30)
    //         .set_left_and_bottom_label_area_size(30.)
    //         .build_cartesian_2d(-100.0..100.0, -100.0..100.0)?;

    //     // axes label and line color
    //     chart
    //         .configure_mesh()
    //         .label_style(("sans-serif", 15).into_font().color(&GREEN))
    //         .axis_style(&GREEN)
    //         .draw()?;

    //     let cs = chart.into_chart_state();
    //     // root.present()?;
    //     cs
    // };

    let mut data: VecDeque<(f64, na::SVector<f64, 3>)> = VecDeque::new();

    let model = bicycle_kinematic::Model::new(1.0, 1.0);
    let mut current_state: na::SVector<f64, 3> = na::SVector::<f64, 3>::zeros();
    let mut current_input: na::SVector<f64, 2> =
        na::SVector::<f64, 2>::new(4., f64::consts::FRAC_PI_8);

    let start_time = SystemTime::now();
    let mut last_flushed = 0.;
    let sample_step: f64 = 1.0 / SAMPLE_RATE;
    let frame_step: f64 = 1.0 / FPS;

    while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
        let epoch = SystemTime::now()
            .duration_since(start_time)
            .unwrap()
            .as_secs_f64();

        if let Some((ts, _)) = data.back() {
            if epoch - ts < 1.0 / SAMPLE_RATE {
                std::thread::sleep(std::time::Duration::from_secs_f64(epoch - ts));
                continue;
            }
            let mut ts = *ts;
            while ts < epoch {
                let keys = window.get_keys_pressed(KeyRepeat::Yes);

                for key in keys {
                    match key {
                        Key::Up => {
                            current_input[0] += 0.1;
                        }
                        Key::Down => {
                            current_input[0] -= 0.1;
                        }
                        Key::Left => {
                            current_input[1] += 0.1;
                        }
                        Key::Right => {
                            current_input[1] -= 0.1;
                        }
                        _ => {
                            continue;
                        }
                    }
                }
                current_state = model.propagate(
                    &current_state,
                    &current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4,
                );
                ts += sample_step;

                data.push_back((ts, current_state));
            }
        }

        // add data
        // TODO: disingenuous cuz how will sample_step be the time step if using system time?
        // might go away from system time
        current_state = model.propagate(
            &current_state,
            &current_input,
            epoch,
            sample_step,
            &runge_kutta::rk4,
        );
        data.push_back((epoch, current_state));

        if epoch - last_flushed > frame_step {
            while data.len() > 20 {
                data.pop_front();
            }

            {
                let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
                    buf.borrow_mut(),
                    (WIDTH as u32, HEIGHT as u32),
                )
                .unwrap()
                .into_drawing_area();
                // root.fill(&BLACK).unwrap();

                let mut chart = cs.clone().restore(&root);
                chart.plotting_area().fill(&BLACK).unwrap();

                chart
                    .configure_mesh()
                    .bold_line_style(&GREEN.mix(0.2))
                    .light_line_style(&TRANSPARENT)
                    .draw()
                    .unwrap();
                chart
                    .draw_series(data.iter().zip(data.iter().skip(1)).map(
                        |(&(t0, x0), &(t1, x1))| {
                            PathElement::new(vec![(x0[0], x0[1]), (x1[0], x1[1])], &GREEN)
                        },
                    ))
                    .unwrap();
            }

            window.update_with_buffer(buf.borrow(), WIDTH, HEIGHT)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}

// plot::update_2d_chartstate(
//     window,
//     buf,
//     cs,
//     runge_kutta::rk4,
//     model,
//     data,
//     current_state,
//     current_input,
//     SAMPLE_RATE,
//     FPS,
//     WIDTH as u32,
//     HEIGHT as u32,
//     10,
//     30,
//     "sans-serif",
//     15,
//     &plotters::style::RGBColor(0u8, 255u8, 0u8),
//     [-100.0, 100.0],
//     [-100.0, 100.0],
// );
