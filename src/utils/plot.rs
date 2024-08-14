#[allow(unused)]
// rust robotics
use crate::models::base;

// 3rd party or std
use minifb;
use plotters::prelude::*;
use plotters::{chart::ChartState, coord::types::RangedCoordf64};
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use serde::Deserialize;
use std::collections::HashMap;

/// Parameters to make a window
#[derive(Debug, Deserialize)]
pub struct WindowParams {
    pub title: String,
    pub width: usize,
    pub height: usize,
}

/// Parameters to make a chart
#[derive(Debug, Deserialize)]
pub struct ChartParams {
    pub background_color: [u8; 3], // RGB
    pub label_color: [u8; 3],      // RGB
    pub margin: i32,
    pub label_size: i32,
    pub label_font: String,
    pub label_font_size: i32,
    pub x_range: [f64; 2],
    pub y_range: [f64; 2],
}

#[allow(unused)]
pub fn create_title(title: &str, keyboard_control: &HashMap<minifb::Key, f64>) -> String {
    // "{title}".to_string()
    todo!();
}

#[allow(unused)]
pub fn create_window(window_params: &WindowParams) -> Result<minifb::Window, minifb::Error> {
    minifb::Window::new(
        &window_params.title as &str,
        window_params.width,
        window_params.height,
        minifb::WindowOptions::default(),
    )
}

#[allow(unused)]
pub fn create_2d_chartstate(
    buf: &mut [u8],
    window_params: &WindowParams,
    chart_params: &ChartParams,
) -> ChartState<Cartesian2d<RangedCoordf64, RangedCoordf64>> {
    // TODO: remove unwrap and make the output result because this assumes happy path
    let background_color = &plotters::style::RGBColor(
        chart_params.background_color[0],
        chart_params.background_color[1],
        chart_params.background_color[2],
    );

    let label_color = &plotters::style::RGBColor(
        chart_params.label_color[0],
        chart_params.label_color[1],
        chart_params.label_color[2],
    );

    // Expects the buffer to mutable. I think this is to change the pixel values in each buffer
    let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
        buf,
        (window_params.width as u32, window_params.height as u32),
    )
    .unwrap()
    .into_drawing_area();

    // Establish the background color
    root.fill(background_color).unwrap();

    // Chart builds on top of the root drawing backend
    let mut chart = ChartBuilder::on(&root)
        .margin(chart_params.margin)
        .set_left_and_bottom_label_area_size(chart_params.label_size)
        .build_cartesian_2d(
            chart_params.x_range[0]..chart_params.x_range[1],
            chart_params.y_range[0]..chart_params.y_range[1],
        )
        .unwrap();

    // Establish the chart styles
    chart
        .configure_mesh()
        .label_style(
            (
                &chart_params.label_font as &str,
                chart_params.label_font_size,
            )
                .into_font()
                .color(label_color),
        )
        .axis_style(label_color)
        .draw()
        .unwrap();

    // Converts ChartContext to ChartState to use for later
    chart.into_chart_state()
}
