// rust robotics
use rust_robotics::controls::path_tracking;
use rust_robotics::models::base::System;
use rust_robotics::models::ground_vehicles::bicycle_kinematic;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::convert;
use rust_robotics::utils::defs;
use rust_robotics::utils::files;
use rust_robotics::utils::math;
use rust_robotics::utils::plot2;

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

// Conversions
const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

// initial states
const VEL_INIT: f64 = 1.0;
const RWA_INIT: f64 = 0.0;
const VEL_STEP: f64 = 0.1;
const RWA_STEP: f64 = 0.5 * DEG_TO_RAD;
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

    // Obtain config_path from command line
    // TODO: make a class for this to streamline this and send helpful error messages
    let animate_cfg_path = &args[1] as &str;
    let bike_cfg_path = &args[2] as &str;

    // Obtain plot config params given the file
    let plot_config: plot2::Config = files::read_toml(animate_cfg_path);

    let chart_params: plot2::ChartParams = plot_config.chart_params;
    let mut window_params: plot2::WindowParams = plot_config.window_params;
    window_params.title = get_window_title(VEL_INIT, RWA_INIT);
    let animation_params: plot2::AnimationParams = plot_config.animation_params;

    let mut buf = defs::BufferWrapper(vec![0u32; window_params.width * window_params.height]);

    let mut window = plot2::create_window(&window_params)?;

    let cs = plot2::create_2d_chartstate(buf.borrow_mut(), &window_params, &chart_params);

    // csv readers and writers
    let mut csv_wtr = csv::Writer::from_path("logs/examples/example_path.csv")?;

    // Could do it this way where a model is initialized and then read using the model
    // let mut model = bicycle_kinematic::Model::new(1.0, 1.0);
    // model.read(bike_cfg_path);
    let model: bicycle_kinematic::Model = files::read_toml(bike_cfg_path);
    let mut current_state: na::SVector<f64, 3> = na::SVector::<f64, 3>::zeros();
    let mut current_input: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(VEL_INIT, RWA_INIT);
    let mut data: VecDeque<(f64, na::SVector<f64, 3>, na::SVector<f64, 2>)> = VecDeque::new();

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

                csv_wtr.serialize(path_tracking::RecordPoint::new(
                    ts,
                    current_state[0],
                    current_state[1],
                    0.,
                ))?;
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
        csv_wtr.serialize(path_tracking::RecordPoint::new(
            epoch,
            current_state[0],
            current_state[1],
            0.,
        ))?;
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

                chart.draw_series(data.iter().zip(data.iter().skip(1)).map(
                    |(&(_t0, x0, _u0), &(_t1, x1, _u1))| {
                        PathElement::new(
                            vec![(x0[0], x0[1]), (x1[0], x1[1])],
                            &chart_params.create_label_color(),
                        )
                    },
                ))?;

                // TODO: cleaner way to do this? Shouldn't really need a for loop
                // remove unwrap
                {
                    // data.back().iter().map(|&(_t0, x0, u0)| {
                    // TODO: magic numbers
                    let vehicle_points = math::calculate_rectangle_points(
                        &(data.back().unwrap().1[0], data.back().unwrap().1[1]),
                        model.get_length_front(),
                        model.get_length_rear(),
                        0.9,
                        0.9,
                        data.back().unwrap().1[2],
                        defs::AngleUnits::Radian,
                    );
                    chart.draw_series(std::iter::once(plot2::polygon_element(
                        &vehicle_points.to_vec(),
                        &chart_params.label_color,
                    )))?;

                    for i in 0..vehicle_points.len() {
                        // TODO: magic numbers
                        // TODO:cleaner way to have front wheels change wheel direction?
                        // not nice with the needing of index
                        let total_heading = if i < 2 {
                            data.back().unwrap().1[2] + data.back().unwrap().2[1]
                        } else {
                            data.back().unwrap().1[2]
                        };

                        let tire_points = math::calculate_rectangle_points(
                            &vehicle_points[i],
                            0.5,
                            0.5,
                            0.25,
                            0.25,
                            total_heading,
                            defs::AngleUnits::Radian,
                        );
                        chart.draw_series(std::iter::once(plot2::polygon_filled_element(
                            &tire_points.to_vec(),
                            &chart_params.label_color,
                        )))?;
                    }
                }

                // easy way to plot one element
                chart.draw_series(data.back().iter().map(|&(_t0, x0, _u0)| {
                    let end_points = math::calculate_line_endpoints(
                        &(x0[0], x0[1]),
                        3.,
                        x0[2],
                        defs::AngleUnits::Radian,
                    );
                    plot2::arrow_element(&end_points, &(255u8, 255u8, 255u8))
                }))?;
            }

            window.set_title(&get_window_title(current_input[0], current_input[1]));

            window.update_with_buffer(buf.borrow(), window_params.width, window_params.height)?;
            last_flushed = epoch;
        }
    }

    csv_wtr.flush()?;
    Ok(())
}
