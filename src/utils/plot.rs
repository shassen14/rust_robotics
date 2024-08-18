#[allow(unused)]
// rust robotics
use crate::utils::defs;
use crate::utils::transforms::FrameTransform3;

// 3rd party or std
use minifb;
use nalgebra as na;
use plotters::prelude::*;
use plotters::{chart::ChartState, coord::types::RangedCoordf64};
use plotters_arrows;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use serde::Deserialize;
use std::collections::HashMap;

#[derive(Deserialize)]
pub struct Config {
    pub window_params: WindowParams,
    pub chart_params: ChartParams,
}
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
    pub background_color: (u8, u8, u8), // RGB
    pub label_color: (u8, u8, u8),      // RGB
    pub margin: i32,
    pub label_size: i32,
    pub label_font: String,
    pub label_font_size: i32,
    pub x_range: [f64; 2],
    pub y_range: [f64; 2],
}

impl ChartParams {
    pub fn create_background_color(&self) -> RGBColor {
        plotters::style::RGBColor(
            self.background_color.0,
            self.background_color.1,
            self.background_color.2,
        )
    }
    pub fn create_label_color(&self) -> RGBColor {
        plotters::style::RGBColor(self.label_color.0, self.label_color.1, self.label_color.2)
    }
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
        chart_params.background_color.0,
        chart_params.background_color.1,
        chart_params.background_color.2,
    );

    let label_color = &plotters::style::RGBColor(
        chart_params.label_color.0,
        chart_params.label_color.1,
        chart_params.label_color.2,
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

#[allow(unused)]
pub fn rectangle_element(
    points: &[(f64, f64); 4],
    color: &(u8, u8, u8),
) -> PathElement<(f64, f64)> {
    // TODO: cleaner way to do this?
    let rect_points: [(f64, f64); 5] = [points[0], points[1], points[2], points[3], points[0]];

    // Color
    let rect_color = &plotters::style::RGBColor(color.0, color.1, color.2);

    PathElement::new(rect_points, &rect_color)
}

#[allow(unused)]
pub fn rectangle_filled_element(
    points: &[(f64, f64); 4],
    color: &(u8, u8, u8),
) -> Polygon<(f64, f64)> {
    // TODO: cleaner way to do this?
    let rect_points: [(f64, f64); 5] = [points[0], points[1], points[2], points[3], points[0]];

    // Color
    let rect_color = &plotters::style::RGBColor(color.0, color.1, color.2);

    Polygon::new(rect_points, &rect_color)
}
#[allow(unused)]
pub fn arrow_element(
    start_point: &(f64, f64),
    length: f64,
    heading_angle: f64,
    angle_units: defs::AngleUnits,
    color: &(u8, u8, u8),
) -> plotters_arrows::ThinArrow<(f64, f64), i32> {
    let tf = FrameTransform3::new(
        &[start_point.0, start_point.1, 0., 0., 0., heading_angle],
        angle_units,
    );
    let end_point = tf.point_b_to_i(&na::Point3::new(length, 0., 0.));

    let arrow_color = &plotters::style::RGBColor(color.0, color.1, color.2);

    plotters_arrows::ThinArrow::new(
        (start_point.0, start_point.1),
        (end_point[0], end_point[1]),
        &arrow_color,
    )
}
