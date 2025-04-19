// rust robotics
use rust_robotics::controls::path_tracking;
use rust_robotics::controls::pid;
use rust_robotics::models::base::System;
use rust_robotics::models::ground_vehicles::bicycle_kinematic;
use rust_robotics::models::ground_vehicles::longitudinal_dynamics;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::constant;
use rust_robotics::utils::convert;
use rust_robotics::utils::defs;
use rust_robotics::utils::files;
use rust_robotics::utils::math;
use rust_robotics::utils::plot2;

// 3rd party or std
use nalgebra as na;
use plotters::prelude::*;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::env;
use std::error::Error;
use std::time::SystemTime;

// initial states
const VEL_INIT: f64 = 0.0;
const RWA_INIT: f64 = 0.0;
const VEL_UPPER_BOUND: f64 = 20.0; // m/s
const VEL_LOWER_BOUND: f64 = 2.0; // m/s
const RWA_UPPER_BOUND: f64 = 40.0; // deg
const RWA_LOWER_BOUND: f64 = -40.0; // deg

const MU: f64 = 0.3;
const EPS: f64 = 1e-6;

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
    // Obtain path and also calculate speed profile
    let mut csv_rdr = csv::Reader::from_path("logs/examples/example_path.csv")?;
    let mut pth: Vec<na::Point3<f64>> = vec![];
    for result in csv_rdr.deserialize() {
        let record: path_tracking::RecordPoint<f64> = result?;
        pth.push(na::Point3::new(record.x, record.y, record.z));
    }

    // 1. Calculate path length
    let n = pth.len();
    let vec_x = pth.iter().map(|p| p.x).collect::<Vec<f64>>();
    let vec_y = pth.iter().map(|p| p.y).collect::<Vec<f64>>();
    let s = math::compute_arc_length(&vec_x, &vec_y);

    // println!("vec_x: {:.2}, s: {:.2}, ", vec_x.len(), s.len());

    // 2. Compute derivatives
    let dx = math::gradient_1d(&vec_x, &s);
    let dy = math::gradient_1d(&vec_y, &s);

    // println!("dx: {:.2}, dy: {:.2}", dx.len(), dy.len());

    let ddx = math::gradient_1d(&dx, &s);
    let ddy = math::gradient_1d(&dy, &s);

    // println!("ddx: {:.2}, ddy: {:.2}", ddx.len(), ddy.len());

    // 3. Compute curvature
    let mut curvature = Vec::with_capacity(n);
    for i in 0..n {
        let num = dx[i] * ddy[i] - dy[i] * ddx[i];
        let denom = (dx[i].powi(2) + dy[i].powi(2)).powf(1.5);
        curvature.push(num / denom);
    }

    // 4. Compute speed profile
    let mut v_profile = Vec::with_capacity(n);
    for i in 0..n {
        let v = (MU * constant::gravity::<f64>() / (curvature[i].abs() + EPS)).sqrt();
        v_profile.push(v.min(VEL_UPPER_BOUND).max(VEL_LOWER_BOUND));
    }

    // println!("v_profile: {:.2}", v_profile.len());

    // 5. Compute acceleration profile from speed profile
    let dv_ds = math::gradient_1d(&v_profile, &s);

    // println!("dv_ds: {:.2}", dv_ds.len());

    let mut a_profile = Vec::with_capacity(n);
    for i in 0..n - 1 {
        let a = dv_ds[i] * v_profile[i];
        a_profile.push(a);
    }

    // println!("dv_ds: {dv_ds:.2?}");
    // println!("s: {s:.2?}");
    // println!("a_profile: {a_profile:.2?}");
    // println!("v_profile: {v_profile:.2?}");

    let lookahead_distance = 5.0;
    let mut start_index = 0usize;
    let mut target_distance: f64;
    let mut target_point = na::Point3::new(0., 0., 0.);

    // Simple PID controller for velocity
    let mut long_controller_fb = pid::Controller::new(
        vec![2000.0],
        vec![1.0],
        vec![0.0],
        vec![-200.0; 1],
        vec![200.0; 1],
    );

    // Longitudinal Model (kg, m^2)
    // TODO: magic numbers, use config
    let long_model: longitudinal_dynamics::Model = longitudinal_dynamics::Model::new(1200., 2.2);
    let mut long_current_state: na::SVector<f64, 1> = na::SVector::<f64, 1>::new(VEL_INIT);
    let mut long_current_input: na::SVector<f64, 1> = na::SVector::<f64, 1>::new(0.);
    let mut long_data: VecDeque<(f64, na::SVector<f64, 1>, na::SVector<f64, 1>)> = VecDeque::new();

    // Lateral Model
    // Could do it this way where a model is initialized and then read using the model
    // let mut model = bicycle_kinematic::Model::new(1.0, 1.0);
    // model.read(bike_cfg_path);
    let lat_model: bicycle_kinematic::Model = files::read_toml(bike_cfg_path);
    let mut lat_current_state: na::SVector<f64, 3> =
        na::SVector::<f64, 3>::new(0.0, 10.0, convert::deg_to_rad(-45.0));
    let mut lat_current_input: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(VEL_INIT, RWA_INIT);
    let mut lat_data: VecDeque<(f64, na::SVector<f64, 3>, na::SVector<f64, 2>)> = VecDeque::new();

    let start_time = SystemTime::now();
    let mut last_flushed = 0.;
    let sample_step: f64 = 1.0 / animation_params.sample_rate;
    let frame_step: f64 = 1.0 / animation_params.frame_rate;

    while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
        let epoch = SystemTime::now().duration_since(start_time)?.as_secs_f64();

        if let Some((ts, _, _)) = lat_data.back() {
            if epoch - ts < 1.0 / animation_params.sample_rate {
                std::thread::sleep(std::time::Duration::from_secs_f64(epoch - ts));
                continue;
            }
            let mut ts = *ts;
            while ts < epoch {
                (start_index, target_distance, target_point) =
                    path_tracking::calculate_lookahead_point(
                        &pth,
                        &na::Point3::new(lat_current_state[0], lat_current_state[1], 0.),
                        start_index,
                        lookahead_distance,
                    );

                if start_index == pth.len() - 1 {
                    start_index = 0;
                }

                let target_vel = v_profile[start_index];
                // let target_vel = VEL_DESIRED;
                let current_vel = long_current_state[0];

                long_current_input = na::SVector::<f64, 1>::from_vec(
                    long_controller_fb.compute(&vec![target_vel - current_vel]),
                );

                // println!(
                //     "long_current_input: {long_current_input:.2?}, target_vel: {target_vel:.2?}, current_vel: {current_vel:.2?}"
                // );

                long_current_state = long_model.propagate(
                    &long_current_state,
                    &long_current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4,
                );

                // TODO: cheat to make current long state / velocity bounded

                // lat_current_input[0] =
                //     math::bound_value(long_current_state[0], VEL_LOWER_BOUND, VEL_UPPER_BOUND);

                lat_current_input[0] = long_current_state[0];

                lat_current_input[1] = path_tracking::pure_pursuit(
                    &na::Point2::new(lat_current_state[0], lat_current_state[1]),
                    &target_point.xy(),
                    lat_current_state[2],
                    target_distance,
                    lat_model.get_wheelbase(),
                );

                lat_current_input[1] = math::bound_value(
                    lat_current_input[1],
                    convert::deg_to_rad(RWA_LOWER_BOUND),
                    convert::deg_to_rad(RWA_UPPER_BOUND),
                );
                lat_current_state = lat_model.propagate(
                    &lat_current_state,
                    &lat_current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4,
                );
                ts += sample_step;

                lat_data.push_back((ts, lat_current_state, lat_current_input));
                lat_data.pop_front();
            }
        }

        // add data
        // TODO: disingenuous cuz how will sample_step be the time step if using system time?
        // might go away from system time

        long_current_state = long_model.propagate(
            &long_current_state,
            &long_current_input,
            epoch,
            sample_step,
            &runge_kutta::rk4,
        );

        lat_current_state = lat_model.propagate(
            &lat_current_state,
            &lat_current_input,
            epoch,
            sample_step,
            &runge_kutta::rk4,
        );

        // println!("Long velocity: {:.2} m/s", long_current_state[0]);

        long_data.push_back((epoch, long_current_state, long_current_input));
        lat_data.push_back((epoch, lat_current_state, lat_current_input));

        if epoch - last_flushed > frame_step {
            while lat_data.len() > 2 {
                long_data.pop_front();
                lat_data.pop_front();
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

                chart.draw_series(lat_data.iter().zip(lat_data.iter().skip(1)).map(
                    |(&(_t0, x0, _u0), &(_t1, x1, _u1))| {
                        PathElement::new(
                            vec![(x0[0], x0[1]), (x1[0], x1[1])],
                            &chart_params.create_label_color(),
                        )
                    },
                ))?;

                chart.draw_series(std::iter::once(Circle::new(
                    (target_point.x, target_point.y),
                    2.0,
                    &WHITE,
                )))?;

                chart.draw_series(pth.iter().zip(pth.iter().skip(1)).map(
                    |(&point1, &point2)| {
                        PathElement::new(vec![(point1.x, point1.y), (point2.x, point2.y)], &RED)
                    },
                ))?;

                // TODO: cleaner way to do this? Shouldn't really need a for loop
                // remove unwrap
                {
                    // data.back().iter().map(|&(_t0, x0, u0)| {
                    // TODO: magic numbers
                    let vehicle_points = math::calculate_rectangle_points(
                        &(lat_data.back().unwrap().1[0], lat_data.back().unwrap().1[1]),
                        lat_model.get_length_front(),
                        lat_model.get_length_rear(),
                        0.9,
                        0.9,
                        lat_data.back().unwrap().1[2],
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
                            lat_data.back().unwrap().1[2] + lat_data.back().unwrap().2[1]
                        } else {
                            lat_data.back().unwrap().1[2]
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
                chart.draw_series(lat_data.back().iter().map(|&(_t0, x0, _u0)| {
                    let end_points = math::calculate_line_endpoints(
                        &(x0[0], x0[1]),
                        3.,
                        x0[2],
                        defs::AngleUnits::Radian,
                    );
                    plot2::arrow_element(&end_points, &(255u8, 255u8, 255u8))
                }))?;
            }

            window.set_title(&get_window_title(
                lat_current_input[0],
                lat_current_input[1],
            ));

            window.update_with_buffer(buf.borrow(), window_params.width, window_params.height)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}
