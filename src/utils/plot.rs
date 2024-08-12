#[allow(unused)]
// rust robotics
use crate::models::base;
use crate::num_methods::defs;
use crate::num_methods::runge_kutta;
use crate::utils::defs::BufferWrapper;

// 3rd party or std
use minifb;
use nalgebra as na;
use plotters::prelude::*;
use plotters::{chart::ChartState, coord::types::RangedCoordf64};
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::time::SystemTime;

#[allow(unused)]
pub fn create_title(title: &str, keys: &[&str], values: &[f64]) -> String {
    // "{title}".to_string()
    todo!();
}

#[allow(unused)]
pub fn create_window(title: &str, width: usize, height: usize) -> minifb::Window {
    // TODO: remove unwrap and make the output result because this assumes happy path
    minifb::Window::new(title, width, height, minifb::WindowOptions::default()).unwrap()
}

#[allow(unused)]
pub fn create_2d_chartstate(
    buf: &mut [u8],
    width: u32,
    height: u32,
    margin: i32,
    background_color: &RGBColor,
    label_color: &RGBColor,
    label_size: i32,
    label_font: &str,
    label_font_size: i32,
    x_range: [f64; 2],
    y_range: [f64; 2],
) -> ChartState<Cartesian2d<RangedCoordf64, RangedCoordf64>> {
    // TODO: remove unwrap and make the output result because this assumes happy path

    // Expects the buffer to mutable. I think this is to change the pixel values in each buffer
    let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(buf, (width, height))
        .unwrap()
        .into_drawing_area();

    // Establish the background color
    root.fill(background_color).unwrap();

    // Chart builds on top of the root drawing backend
    let mut chart = ChartBuilder::on(&root)
        .margin(margin)
        .set_left_and_bottom_label_area_size(label_size)
        .build_cartesian_2d(x_range[0]..x_range[1], y_range[0]..y_range[1])
        .unwrap();

    // Establish the chart styles
    chart
        .configure_mesh()
        .label_style(
            (label_font, label_font_size)
                .into_font()
                .color(&label_color),
        )
        .axis_style(&label_color)
        .draw()
        .unwrap();

    // Converts ChartContext to ChartState to use for later
    chart.into_chart_state()
}

// #[allow(unused)]
// TODO: this function is doing a lot, break it up, possibly make it more generic with types,
// need num-traits if I want to go that route most likely
// TODO: OMG SO MANY INPUTS
// TODO: Take a hard look at this horrendous function
// pub fn update_2d_chartstate<const N: usize, const M: usize>(
//     mut window: minifb::Window,
//     mut buf: BufferWrapper,
//     mut chart: ChartState<Cartesian2d<RangedCoordf64, RangedCoordf64>>,
//     integrator: impl defs::IntegrationFn<f64, N>,
//     model: impl base::System<f64, N, M>,
//     mut data: VecDeque<(f64, na::SVector<f64, N>)>,
//     mut current_state: na::SVector<f64, N>,
//     mut current_input: na::SVector<f64, M>,
//     samples_per_sec: f64,
//     frames_per_sec: f64,
//     width: u32,
//     height: u32,
//     margin: i32,
//     label_size: i32,
//     label_font: &str,
//     label_font_size: i32,
//     label_color: &RGBColor,
//     x_range: [f64; 2],
//     y_range: [f64; 2],
// ) -> () {
//     let start_time = SystemTime::now();
//     let mut last_flushed = 0.;
//     let sample_step: f64 = 1.0 / samples_per_sec;
//     let frame_step: f64 = 1.0 / frames_per_sec;

//     while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
//         let epoch = SystemTime::now()
//             .duration_since(start_time)
//             .unwrap()
//             .as_secs_f64();

//         if let Some((ts, _)) = data.back() {
//             if epoch - ts < 1.0 / samples_per_sec {
//                 std::thread::sleep(std::time::Duration::from_secs_f64(epoch - ts));
//                 continue;
//             }
//             let mut ts = *ts;
//             while ts < epoch {
//                 current_state = model.propagate(
//                     &current_state,
//                     &current_input,
//                     epoch,
//                     sample_step,
//                     &integrator,
//                 );
//                 ts += sample_step;

//                 data.push_back((ts, current_state));
//             }
//         }

//         // add data
//         // TODO: disingenuous cuz how will sample_step be the time step if using system time?
//         // might go away from system time
//         current_state = model.propagate(
//             &current_state,
//             &current_input,
//             epoch,
//             sample_step,
//             &integrator,
//         );

//         if epoch - last_flushed > frame_step {
//             let root = BitMapBackend::<RGBPixel>::with_buffer_and_format(
//                 buf.borrow_mut(),
//                 (width, height),
//             )
//             .unwrap()
//             .into_drawing_area();
//             root.fill(&BLACK).unwrap();

//             let mut cs = chart.clone().restore(&root);
//             cs.plotting_area().fill(&BLACK).unwrap();

//             cs.configure_mesh()
//                 .bold_line_style(&GREEN.mix(0.2))
//                 .light_line_style(&TRANSPARENT)
//                 .draw()
//                 .unwrap();
//             cs.draw_series(
//                 data.iter()
//                     .zip(data.iter().skip(1))
//                     .map(|(&(t0, x0), &(t1, x1))| {
//                         PathElement::new(vec![(x0[0], x0[1]), (x1[0], x1[1])], &GREEN)
//                     }),
//             )
//             .unwrap();
//         }
//         window.update_with_buffer(buf.borrow(), width as usize, height as usize);
//     }
//     todo!();
// }
