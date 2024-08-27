// rust robotics
use rust_robotics::controls::pid;
use rust_robotics::models::base::System;
use rust_robotics::models::humanoid::two_joint_arm;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::defs;
use rust_robotics::utils::defs::AngleUnits;
use rust_robotics::utils::files;
use rust_robotics::utils::math;
use rust_robotics::utils::plot;
use rust_robotics::utils::transforms::FrameTransform3;

// 3rd party or std
use core::f64;
use minifb::{Key, KeyRepeat};
use nalgebra as na;
use plotters::prelude::*;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::env;
use std::error::Error;
use std::time::SystemTime;

// Conversions
const LENGTH1: f64 = 3.0;
const LENGTH2: f64 = 2.5;

fn main() -> Result<(), Box<dyn Error>> {
    // Read command line arguments
    let args: Vec<String> = env::args().collect();

    // Obtain config_path from command line
    // TODO: make a class for this to streamline this and send helpful error messages
    let animate_cfg_path = &args[1] as &str;
    // let bike_cfg_path = &args[2] as &str;

    // Obtain plot config params given the file
    let plot_config: plot::Config = files::read_config(animate_cfg_path);

    let chart_params: plot::ChartParams = plot_config.chart_params;
    let mut window_params: plot::WindowParams = plot_config.window_params;
    window_params.title = "Two Joint Robotic Arm".to_string();
    let animation_params: plot::AnimationParams = plot_config.animation_params;

    let mut buf = defs::BufferWrapper(vec![0u32; window_params.width * window_params.height]);

    let mut window = plot::create_window(&window_params)?;

    let cs = plot::create_2d_chartstate(buf.borrow_mut(), &window_params, &chart_params);

    let model: two_joint_arm::Model<f64> = two_joint_arm::Model {
        link_lengths: [LENGTH1, LENGTH2],
    };

    println!("{:?}", model);
    let mut current_state: na::SVector<f64, 8> =
        model.feasible_state_initial([3.1415 / 2.0, 3.1415 / 2.0]);
    let mut current_input: na::SVector<f64, 2> = na::SVector::<f64, 2>::zeros();
    let mut data: VecDeque<(f64, na::SVector<f64, 8>, na::SVector<f64, 2>)> = VecDeque::new();

    let mut controller: pid::Controller<2> = pid::Controller::<2>::new(
        na::SVector::<f64, 2>::new(0.75, 0.75),
        na::SVector::<f64, 2>::new(0.0001, 0.0001),
        na::SVector::<f64, 2>::new(0.01, 0.01),
        na::SVector::<f64, 2>::new(-0.1, -0.1),
        na::SVector::<f64, 2>::new(0.1, 0.1),
    );

    let start_time = SystemTime::now();
    let mut last_flushed = 0.;
    let sample_step: f64 = 1.0 / animation_params.sample_rate;
    let frame_step: f64 = 1.0 / animation_params.frame_rate;

    while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
        let epoch = SystemTime::now().duration_since(start_time)?.as_secs_f64();

        if let Some((ts, _, _)) = data.back() {
            if epoch - ts < 1.0 / animation_params.sample_rate {
                std::thread::sleep(std::time::Duration::from_secs_f64(epoch - ts));
                continue;
            }
            let mut ts = *ts;
            while ts < epoch {
                // let keys = window.get_keys_pressed(KeyRepeat::Yes);
                let mouse_position = if window.get_mouse_down(minifb::MouseButton::Left) {
                    window.get_mouse_pos(minifb::MouseMode::Clamp)
                } else {
                    None
                };

                let mut angle_desired: na::SVector<f64, 2>;
                if mouse_position.is_some() {
                    let offsets = [
                        -376.,
                        348.,
                        0.,
                        f64::consts::PI,
                        0.0 * f64::consts::PI,
                        f64::consts::FRAC_PI_2 * 0.,
                    ];
                    // TODO: Magic numbers need to make this be able to change with the window size and plot axes
                    let transform = FrameTransform3::new(&offsets, AngleUnits::Radian);
                    let p = transform.point_b_to_i(&na::Point3::new(
                        mouse_position.unwrap().0 as f64,
                        mouse_position.unwrap().1 as f64,
                        0.,
                    )) / 67.0;

                    angle_desired = model.inverse_kinematics(&na::SVector::<f64, 2>::new(p.x, p.y));
                } else {
                    angle_desired = na::SVector::<f64, 2>::new(current_state[2], current_state[6]);
                }

                let x_dot_desired = model.calculate_x_dot_desired(&angle_desired, &current_state);
                // current_state[0] = model.link_lengths[0] * f64::cos(angle_desired[0]);
                // current_state[1] = model.link_lengths[1] * f64::sin(angle_desired[0]);
                // current_state[2] = angle_desired[0];

                // current_state[4] = current_state[0]
                //     + model.link_lengths[1] * f64::cos(angle_desired[0] + angle_desired[1]);

                // current_state[5] = current_state[1]
                //     + model.link_lengths[1] * f64::sin(angle_desired[0] + angle_desired[1]);
                // current_state[6] = angle_desired[1];

                // current_input = model.calculate_input(&current_state, &x_dot_desired, ts);
                current_input = controller.compute(&na::SVector::<f64, 2>::new(
                    x_dot_desired[3],
                    x_dot_desired[7],
                ));
                current_state = model.propagate(
                    &current_state,
                    &current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4,
                );
                println!("angle_desired: {}", angle_desired);
                println!("x_dot_desired: {}", x_dot_desired);
                println!("current input: {}", current_input);

                // println!("current_state: {}", current_state);

                ts += sample_step;

                // println!("{}", current_state);
                data.push_back((ts, current_state, current_input));
                data.pop_front();
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
        data.push_back((epoch, current_state, current_input));

        if epoch - last_flushed > frame_step {
            while data.len() > 2 {
                data.pop_front();
            }

            {
                let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
                    buf.borrow_mut(),
                    (window_params.width as u32, window_params.height as u32),
                )?
                .into_drawing_area();

                let mut chart = cs.clone().restore(&root);
                chart.plotting_area().fill(&BLACK)?;

                chart
                    .configure_mesh()
                    .bold_line_style(&chart_params.create_label_color().mix(0.2))
                    .light_line_style(&TRANSPARENT)
                    .draw()?;

                // chart.draw_series(data.iter().zip(data.iter().skip(1)).map(
                //     |(&(_t0, x0, _u0), &(_t1, x1, _u1))| {
                //         PathElement::new(
                //             vec![(x0[0], x0[1]), (x1[0], x1[1])],
                //             &chart_params.create_label_color(),
                //         )
                //     },
                // ))?;

                chart.draw_series(data.back().iter().map(|&(_t0, x0, _u0)| {
                    let end_points = math::calculate_line_endpoints(
                        &(x0[0], x0[1]),
                        LENGTH2,
                        x0[2] + x0[6],
                        defs::AngleUnits::Radian,
                    );
                    PathElement::new(
                        vec![end_points[0], end_points[1]],
                        &RGBColor(255u8, 255u8, 255u8),
                    )
                }))?;

                chart.draw_series(data.back().iter().map(|&(_t0, x0, _u0)| {
                    PathElement::new(
                        vec![(0.0, 0.0), (x0[0], x0[1])],
                        &RGBColor(255u8, 255u8, 255u8),
                    )
                }))?;
            }

            window.set_title("Robotic Arm");

            window.update_with_buffer(buf.borrow(), window_params.width, window_params.height)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}

// fn main() {}
