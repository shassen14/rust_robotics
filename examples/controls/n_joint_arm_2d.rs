// rust robotics
use rust_robotics::controls::pid2;
use rust_robotics::models::base_d::{SystemD, SystemHD};
use rust_robotics::models::humanoid::n_joint_arm2_d;
use rust_robotics::num_methods::runge_kutta;
use rust_robotics::utils::defs;
use rust_robotics::utils::files;
use rust_robotics::utils::plot2;

// 3rd party or std
use core::f64;
use nalgebra as na;
use plotters::prelude::*;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::env;
use std::error::Error;
use std::time::SystemTime;

// Types of 2D joint arm
// Define an enum to wrap all possible models with different N and M values
// enum ModelWrapper<T> {
//     Model1x4(n_joint_arm2::Model<T, 1, 4>),
//     Model2x8(n_joint_arm2::Model<T, 2, 8>),
//     Model3x12(n_joint_arm2::Model<T, 3, 12>),
//     Model4x16(n_joint_arm2::Model<T, 4, 16>),
//     Model5x20(n_joint_arm2::Model<T, 5, 20>),
//     Model6x24(n_joint_arm2::Model<T, 6, 24>),
// }

// // TODO: have a default constructor? that is all zeros?
// // TODO: default value to be what?
// // Should I return an optionsal, probably yes
// fn number_to_model(i: usize) -> ModelWrapper<f64> {
//     let model = match i {
//         1 => ModelWrapper::<f64>::Model1x4(n_joint_arm2::Model {
//             link_lengths: vec![0.],
//         }),
//         2 => ModelWrapper::<f64>::Model2x8(n_joint_arm2::Model {
//             link_lengths: vec![0., 0.],
//         }),
//         3 => ModelWrapper::<f64>::Model3x12(n_joint_arm2::Model {
//             link_lengths: vec![0., 0., 0.],
//         }),
//         4 => ModelWrapper::<f64>::Model4x16(n_joint_arm2::Model {
//             link_lengths: vec![0., 0., 0., 0.],
//         }),
//         5 => ModelWrapper::<f64>::Model5x20(n_joint_arm2::Model {
//             link_lengths: vec![0., 0., 0., 0., 0.],
//         }),
//         6 => ModelWrapper::<f64>::Model6x24(n_joint_arm2::Model {
//             link_lengths: vec![0., 0., 0., 0., 0., 0.],
//         }),
//         _ => ModelWrapper::<f64>::Model1x4(n_joint_arm2::Model {
//             link_lengths: vec![0.],
//         }),
//     };
//     model
// }

// impl ModelWrapper<f64> {
//     fn calculate_feasible_state_initial(model_wrapper: Self, angles: &[f64]) -> Vec<f64> {
//         let foo: Vec<f64> = match model_wrapper {
//             ModelWrapper::Model1x4(model) => <rust_robotics::models::humanoid::n_joint_arm2::Model<
//                 f64,
//                 1,
//                 4,
//             > as SystemH<f64, 1, 4, 2>>::calculate_feasible_state_initial(
//                 &model, &angles
//             )
//             .as_slice()
//             .to_vec(),
//             ModelWrapper::Model2x8(model) => <rust_robotics::models::humanoid::n_joint_arm2::Model<
//                 f64,
//                 2,
//                 8,
//             > as SystemH<f64, 2, 8, 2>>::calculate_feasible_state_initial(
//                 &model, &angles
//             )
//             .as_slice()
//             .to_vec(),
//             ModelWrapper::Model3x12(model) => {
//                 <rust_robotics::models::humanoid::n_joint_arm2::Model<f64, 3, 12> as SystemH<
//                     f64,
//                     3,
//                     12,
//                     2,
//                 >>::calculate_feasible_state_initial(&model, &angles)
//                 .as_slice()
//                 .to_vec()
//             }
//             ModelWrapper::Model4x16(model) => {
//                 <rust_robotics::models::humanoid::n_joint_arm2::Model<f64, 4, 16> as SystemH<
//                     f64,
//                     4,
//                     16,
//                     2,
//                 >>::calculate_feasible_state_initial(&model, &angles)
//                 .as_slice()
//                 .to_vec()
//             }
//             ModelWrapper::Model5x20(model) => {
//                 <rust_robotics::models::humanoid::n_joint_arm2::Model<f64, 5, 20> as SystemH<
//                     f64,
//                     5,
//                     20,
//                     2,
//                 >>::calculate_feasible_state_initial(&model, &angles)
//                 .as_slice()
//                 .to_vec()
//             }
//             ModelWrapper::Model6x24(model) => {
//                 <rust_robotics::models::humanoid::n_joint_arm2::Model<f64, 6, 24> as SystemH<
//                     f64,
//                     6,
//                     24,
//                     2,
//                 >>::calculate_feasible_state_initial(&model, &angles)
//                 .as_slice()
//                 .to_vec()
//             }
//         };
//         foo
//     }
// }

// // TODO: have a default constructor? that is all zeros?
// // TODO: default value to be what?
// // Should I return an optionsal, probably yes
// fn model_variant_to_model(variant: ModelWrapper<f64>) -> Box<dyn Any> {
//     let model = match variant {
//         ModelWrapper::Model1x4(model) => Box::new(model),
//         ModelWrapper::Model2x8(model) => Box::new(model),
//         ModelWrapper::Model3x12(model) => Box::new(model),
//         ModelWrapper::Model4x16(model) => Box::new(model),
//         ModelWrapper::Model5x20(model) => Box::new(model),
//         ModelWrapper::Model6x24(model) => Box::new(model),
//         // _ => n_joint_arm2::Model::<f64, 1, 4> {
//         //     link_lengths: vec![0.],
//         // },
//     };
//     model
// }

fn main() -> Result<(), Box<dyn Error>> {
    // Read command line arguments
    let args: Vec<String> = env::args().collect();

    // Obtain config_path from command line
    // TODO: make a class for this to streamline this and send helpful error messages
    let animate_cfg_path = &args[1] as &str;
    let n_joint_cfg_path = &args[2] as &str;

    // Obtain plot config params given the file
    let plot_config: plot2::Config = files::read_toml(animate_cfg_path);
    let n_joint_config: n_joint_arm2_d::NJointArmConfig<f64> = files::read_toml(n_joint_cfg_path);

    // Acquire animation params
    let chart_params: plot2::ChartParams = plot_config.chart_params;
    let window_params: plot2::WindowParams = plot_config.window_params;
    let animation_params: plot2::AnimationParams = plot_config.animation_params;

    let mut buf = defs::BufferWrapper(vec![0u32; window_params.width * window_params.height]);

    let mut window = plot2::create_window(&window_params)?;

    let cs = plot2::create_2d_chartstate(buf.borrow_mut(), &window_params, &chart_params);

    // Acquire robot arm params
    let n_joint_params: n_joint_arm2_d::NJointArmParams<f64> = n_joint_config.n_joint_arm_params;

    let num_inputs = n_joint_params.link_lengths.len();
    let model = n_joint_arm2_d::ModelD {
        link_lengths: n_joint_params.link_lengths,
    };
    let num_states = num_inputs * model.num_states_to_num_dim_ratio() as usize;

    println!("{:?}", model);
    let mut current_state: na::DVector<f64> =
        <rust_robotics::models::humanoid::n_joint_arm2_d::ModelD<f64> as SystemHD<
            f64,
            2,
        >>::calculate_feasible_state_initial(&model, &n_joint_params.link_angles);

    let mut current_input: na::DVector<f64> = na::DVector::<f64>::zeros(num_inputs);
    let mut data: VecDeque<(f64, na::DVector<f64>, na::DVector<f64>)> = VecDeque::new();

    let mut controller: pid2::Controller = pid2::Controller::new(
        vec![2.5, 2.5, 2.5, 2.5, 1.5, 1.5, 1.5],
        vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        vec![0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
        vec![-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1],
        vec![0.1, 0.1, 0.1, 0.1, 0.1, -0.1, 0.1],
    );

    let start_time = SystemTime::now();
    let mut last_flushed = 0.;
    let sample_step: f64 = 1.0 / animation_params.sample_rate;
    let frame_step: f64 = 1.0 / animation_params.frame_rate;

    let mut angle_desired: Vec<f64>;
    let mut mouse_chart_position: (f64, f64) =
        (current_state[num_states - 4], current_state[num_states - 3]);

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

                if mouse_position.is_some() {
                    mouse_chart_position = plot2::mouse_chart_position(
                        (
                            mouse_position.unwrap().0 as f64,
                            mouse_position.unwrap().1 as f64,
                        ),
                        &window_params,
                        &chart_params,
                    );

                    // println!("{:?}", mouse_chart_position);
                }
                angle_desired = model.inverse_kinematics(
                    vec![mouse_chart_position.0, mouse_chart_position.1],
                    &current_state,
                );
                // let x_dot_desired = model.calculate_x_dot_desired(&angle_desired, &current_state);

                // current_input = model.calculate_input(&current_state, &x_dot_desired, ts);
                current_input = na::DVector::<f64>::from_vec(controller.compute(&angle_desired));

                current_state = model.propagate(
                    &current_state,
                    &current_input,
                    epoch,
                    sample_step,
                    &runge_kutta::rk4d,
                );

                ts += sample_step;

                data.push_back((ts, current_state.clone(), current_input.clone()));
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
            &runge_kutta::rk4d,
        );
        data.push_back((epoch, current_state.clone(), current_input.clone()));

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

                if let Some((_, x, _)) = data.back() {
                    let mut previous_point = (0.0, 0.0);
                    let mut link_points: Vec<(f64, f64)> = vec![];

                    for i in 0..num_inputs {
                        link_points.push(previous_point);
                        chart.draw_series(std::iter::once(PathElement::new(
                            vec![previous_point, (x[4 * i], x[4 * i + 1])],
                            &RGBColor(255u8, 255u8, 255u8),
                        )))?;
                        chart.draw_series(std::iter::once(Circle::new(
                            previous_point,
                            5,
                            &RGBColor(0u8, 255u8, 255u8),
                        )))?;
                        previous_point = (x[4 * i], x[4 * i + 1]);
                    }
                    link_points.push(previous_point);
                }

                chart.draw_series(std::iter::once(Cross::new(
                    mouse_chart_position,
                    5,
                    &RGBColor(255, 0, 0),
                )))?;
            }

            window.update_with_buffer(buf.borrow(), window_params.width, window_params.height)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}

// fn main() {}
