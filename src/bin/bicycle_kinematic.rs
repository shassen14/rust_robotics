// rust robotics
use rust_robotics::models::base::System;
use rust_robotics::models::bicycle_kinematic;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::convert;
use rust_robotics::utils::defs;
use rust_robotics::utils::math;
use rust_robotics::utils::plot;

// 3rd party or std
use minifb::{Key, KeyRepeat};
use nalgebra as na;
use plotters::prelude::*;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::env;
use std::error::Error;
use std::time::SystemTime;

use std::fs;
use std::process::exit;

// Conversions
const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

// TODO: read from config yml or some type of file for runtime instead of compile time
// Plot Params
const WIDTH: usize = 720;
const HEIGHT: usize = 720;

// Animation Params
const SAMPLE_RATE: f64 = 100f64;
const FPS: f64 = 30f64;

// initial states
const VEL_INIT: f64 = 1.0;
const RWA_INIT: f64 = 0.0;
const VEL_STEP: f64 = 0.1;
const RWA_STEP: f64 = 1.0 * DEG_TO_RAD;
const VEL_UPPER_BOUND: f64 = 20.0; // m/s
const VEL_LOWER_BOUND: f64 = 0.0; // m/s
const RWA_UPPER_BOUND: f64 = 40.0; // deg
const RWA_LOWER_BOUND: f64 = -40.0; // deg

fn get_window_title(velocity: f64, rwa: f64) -> String {
    format!(
        "velocity={:.1}m/s, rwa={:.1}deg up/down=Adjust vel left/right=Adjust rwa <Esc>=Exit",
        velocity,
        convert::rad_to_deg(rwa)
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    // Read command line arguments
    let args: Vec<String> = env::args().collect();

    // obtain config_path
    let config_path = &args[1] as &str;
    println!("Attempting to read from {}", config_path);

    let contents = match fs::read_to_string(config_path) {
        // If successful return the files text as `contents`.
        // `c` is a local variable.
        Ok(c) => c,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Could not read file `{}`", config_path);
            // Exit the program with exit code `1`.
            exit(1);
        }
    };

    // Use a `match` block to return the
    // file `contents` as a `Data struct: Ok(d)`
    // or handle any `errors: Err(_)`.
    let plot_config: plot::Config = match toml::from_str(&contents) {
        // If successful, return data as `Data` struct.
        // `d` is a local variable.
        Ok(d) => d,
        // Handle the `error` case.
        Err(_) => {
            // Write `msg` to `stderr`.
            eprintln!("Unable to load data from `{}`", config_path);
            // Exit the program with exit code `1`.
            exit(1);
        }
    };

    let window_params: plot::WindowParams = plot::WindowParams {
        title: get_window_title(VEL_INIT, RWA_INIT),
        width: WIDTH,
        height: HEIGHT,
    };

    let chart_params: plot::ChartParams = plot_config.chart_params;

    let mut buf = defs::BufferWrapper(vec![0u32; window_params.width * window_params.height]);

    let mut window = plot::create_window(&window_params)?;

    // TODO: magic numbers
    let cs = plot::create_2d_chartstate(buf.borrow_mut(), &window_params, &chart_params);

    let mut data: VecDeque<(f64, na::SVector<f64, 3>, na::SVector<f64, 2>)> = VecDeque::new();

    let model = bicycle_kinematic::Model::new(1.0, 1.0);
    let mut current_state: na::SVector<f64, 3> = na::SVector::<f64, 3>::zeros();
    let mut current_input: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(VEL_INIT, RWA_INIT);

    let start_time = SystemTime::now();
    let mut last_flushed = 0.;
    let sample_step: f64 = 1.0 / SAMPLE_RATE;
    let frame_step: f64 = 1.0 / FPS;

    while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
        let epoch = SystemTime::now().duration_since(start_time)?.as_secs_f64();

        if let Some((ts, _, _)) = data.back() {
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
                            current_input[0] += VEL_STEP;
                        }
                        Key::Down => {
                            current_input[0] -= VEL_STEP;
                        }
                        Key::Left => {
                            current_input[1] += RWA_STEP;
                        }
                        Key::Right => {
                            current_input[1] -= RWA_STEP;
                        }
                        _ => {
                            continue;
                        }
                    }
                }
                current_input[0] =
                    math::bound_value(current_input[0], VEL_LOWER_BOUND, VEL_UPPER_BOUND);
                current_input[1] = math::bound_value(
                    current_input[1],
                    convert::deg_to_rad(RWA_LOWER_BOUND),
                    convert::deg_to_rad(RWA_UPPER_BOUND),
                );
                current_state = model.propagate(
                    &current_state,
                    &current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4,
                );
                ts += sample_step;

                data.push_back((ts, current_state, current_input));
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
            while data.len() > 200 {
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

                chart.draw_series(data.iter().zip(data.iter().skip(1)).map(
                    |(&(_t0, x0, _u0), &(_t1, x1, _u1))| {
                        PathElement::new(
                            vec![(x0[0], x0[1]), (x1[0], x1[1])],
                            &chart_params.create_label_color(),
                        )
                    },
                ))?;
            }

            window.set_title(&get_window_title(current_input[0], current_input[1]));

            window.update_with_buffer(buf.borrow(), WIDTH, HEIGHT)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}
