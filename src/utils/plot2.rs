#[allow(unused)]
// rust robotics
use crate::utils::defs;

// 3rd party or std
use minifb;
use num_traits::Float;
use plotters::prelude::*;
use plotters::{chart::ChartState, coord::types::RangedCoordf64};
use plotters_arrows;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use serde::Deserialize;
use std::collections::HashMap;

use crate::utils::defs::AngleUnits;
use crate::utils::transforms::FrameTransform3;
use nalgebra as na;

#[derive(Deserialize)]
pub struct Config {
    pub window_params: WindowParams,
    pub chart_params: ChartParams,
    pub animation_params: AnimationParams,
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

/// Parameters to make a window
#[derive(Debug, Deserialize)]
pub struct AnimationParams {
    pub sample_rate: f64,
    pub frame_rate: f64,
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

pub fn mouse_chart_position(
    position: (f64, f64),
    window_params: &WindowParams,
    chart_params: &ChartParams,
) -> (f64, f64) {
    // assuming second value is greater than the first
    assert!(chart_params.x_range[0] < chart_params.x_range[1]);
    assert!(chart_params.y_range[0] < chart_params.y_range[1]);

    // [-1, 1] percentage for the chart's offset depending on the axes range
    let x0_range_percentage =
        -chart_params.x_range[0] / (chart_params.x_range[1] - chart_params.x_range[0]);
    let y0_range_percentage =
        -chart_params.y_range[0] / (chart_params.y_range[1] - chart_params.y_range[0]);

    // x and y offsets dependent on window size, label size, and margin
    let x_offset: f64 = window_params.width as f64 * x0_range_percentage
        + chart_params.label_size as f64 * (1. - x0_range_percentage)
        + chart_params.margin as f64 * (1. - 2.0 * x0_range_percentage);
    let y_offset: f64 = window_params.height as f64 * (1. - y0_range_percentage)
        - chart_params.label_size as f64 * (1. - y0_range_percentage)
        - chart_params.margin as f64 * (1. - 2.0 * y0_range_percentage);

    // convert window axes (+x right, +y down, +z into the page) to +x right, +y up, +z out of the page
    // and translate it to the 0, 0 coordinate on the plot
    let offsets = [-x_offset, y_offset, 0., std::f64::consts::PI, 0., 0.];
    let transform = FrameTransform3::new(&offsets, Some(AngleUnits::Radian));
    let mut p = transform.point_b_to_i(&na::Point3::new(position.0, position.1, 0.));

    // assuming linear graph axes, convert pixel to axes units
    p.x /= (window_params.width as f64
        - chart_params.label_size as f64
        - chart_params.margin as f64 * 2.0)
        / (chart_params.x_range[1] - chart_params.x_range[0]);
    p.y /= (window_params.height as f64
        - chart_params.label_size as f64
        - chart_params.margin as f64 * 2.0)
        / (chart_params.y_range[1] - chart_params.y_range[0]);
    (p.x, p.y)
}

#[allow(unused)]
/// Outputs a PathElement where it is a shape
///
/// * Arguments
///
/// * `points` - The 4 points to plot (must be ordered where point 0 -> 1 -> 2, etc)
/// * `color` - RGB values [0,255] to color the shape
/// * Returns a PathElement of the shape
///
pub fn polygon_element(points: &Vec<(f64, f64)>, color: &(u8, u8, u8)) -> PathElement<(f64, f64)> {
    // TODO: cleaner way to do this?
    let mut polygon_points: Vec<(f64, f64)> = points.to_vec();
    polygon_points.push(points[0]);

    // Color
    let polygon_color = &plotters::style::RGBColor(color.0, color.1, color.2);

    PathElement::new(polygon_points, &polygon_color)
}

#[allow(unused)]
/// Outputs a Polygon where it is a shape fully colored
///
/// * Arguments
///
/// * `points` - The 4 points to plot (must be ordered where point 0 -> 1 -> 2, etc)
/// * `color` - RGB values [0,255] to color the shape
/// * Returns a Polygon of the shape
///
pub fn polygon_filled_element(
    points: &Vec<(f64, f64)>,
    color: &(u8, u8, u8),
) -> Polygon<(f64, f64)> {
    // TODO: cleaner way to do this?
    let mut polygon_points: Vec<(f64, f64)> = points.to_vec();
    polygon_points.push(points[0]);

    // Color
    let color = &plotters::style::RGBColor(color.0, color.1, color.2);

    Polygon::new(polygon_points, &color)
}

#[allow(unused)]
/// Outputs a Thin Arrow where it is has two end points (start points to the end)
///
/// * Arguments
///
/// * `end_points` - The 2 oints to plot (must be ordered where point 0 -> 1)
/// * `color` - RGB values [0,255] to color the shape
/// * Returns a ThinArrow
///
pub fn arrow_element(
    end_points: &[(f64, f64); 2],
    color: &(u8, u8, u8),
) -> plotters_arrows::ThinArrow<(f64, f64), i32> {
    let color = &plotters::style::RGBColor(color.0, color.1, color.2);

    plotters_arrows::ThinArrow::new(
        (end_points[0].0, end_points[0].1),
        (end_points[1].0, end_points[1].1),
        &color,
    )
}

#[allow(unused)]
/// Outputs a Thin Arrow where it is has two end points (start points to the end)
///
/// * Arguments
///
/// * `end_points` - The 2 oints to plot (must be ordered where point 0 -> 1)
/// * `color` - RGB values [0,255] to color the shape
/// * Returns a ThinArrow
///
pub fn circle_element<T>(
    center: (T, T),
    radius: T,
    parts: u8,
    color: &(u8, u8, u8),
) -> Polygon<(T, T)>
where
    T: Float,
{
    let color = &plotters::style::RGBColor(color.0, color.1, color.2);

    let angle: T = T::from(2.0 * std::f64::consts::PI).unwrap()
        / T::from(parts).expect("Invalid circle element argument");
    let points: Vec<(T, T)> = (0..parts)
        .into_iter()
        .map(|part| {
            (
                center.0 + radius * T::cos(angle * T::from(part).unwrap()),
                center.1 + radius * T::sin(angle * T::from(part).unwrap()),
            )
        })
        .collect();

    Polygon::new(points, color)
}
