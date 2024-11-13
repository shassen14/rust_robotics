use rust_robotics::utils::files;
use rust_robotics::utils::geometry::Shape2D;
use rust_robotics::utils::map_generator;

use rust_robotics::utils::plot2;

// Plotters
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::RGBPixel;
use plotters_bitmap::BitMapBackend;
use std::env;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Index2D(i32, i32);

#[derive(Clone, Debug)]
struct Position2D(f64, f64);

fn calculate_position(index: &Index2D, min_position: &Position2D, resolution: f64) -> Position2D {
    Position2D(
        resolution * index.0 as f64 + min_position.0,
        resolution * index.1 as f64 + min_position.1,
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    // Read command line arguments
    let args: Vec<String> = env::args().collect();

    // Obtain config_path from command line
    // TODO: make a class for this to streamline this and send helpful error messages
    let animate_cfg_path = &args[1] as &str;
    let map_param_cfg_path = &args[2] as &str;

    // Obtain plot config params given the file
    let plot_config: plot2::Config = files::read_toml(animate_cfg_path);
    let chart_params: plot2::ChartParams = plot_config.chart_params;
    let window_params: plot2::WindowParams = plot_config.window_params;

    let map_params: map_generator::MapGeneratorParams<f64, u8> =
        files::read_toml(map_param_cfg_path);

    println!("map_params: {:?}", map_params);

    let root = BitMapBackend::<RGBPixel>::new(
        "custom_map.png",
        (window_params.width as u32, window_params.height as u32),
    )
    .into_drawing_area();
    root.fill(&BLACK)?;

    let mut chart = ChartBuilder::on(&root)
        .margin(chart_params.margin)
        .set_all_label_area_size(chart_params.label_size)
        .build_cartesian_2d(
            chart_params.x_range[0]..chart_params.x_range[1],
            chart_params.y_range[0]..chart_params.y_range[1],
        )?;

    chart
        .configure_mesh()
        .label_style(("sans-serif", 15).into_font().color(&WHITE))
        .axis_style(&WHITE)
        .draw()?;

    let resolution = 1.0;
    let bottom_left_pos = Position2D(chart_params.x_range[0], chart_params.y_range[0]);

    let circle_obstacles: Vec<(Shape2D<f64>, u8)> =
        map_generator::convert_circle_param_to_struct(&map_params.shape_list.circles);
    let polygon_obstacle: Vec<(Shape2D<f64>, u8)> = Vec::new();
    let obstacles: Vec<(Shape2D<f64>, u8)> = [circle_obstacles, polygon_obstacle].concat();

    println!("obstacles: {:?}", obstacles);

    let grid = map_generator::GridMap2D::new(
        &obstacles,
        &chart_params.x_range,
        &chart_params.y_range,
        resolution,
    );

    // This will be a circle regardless of axis
    for (shape, _cost) in obstacles {
        match shape {
            Shape2D::Circle(circle) => {
                chart.draw_series(std::iter::once(plot2::circle_element(
                    circle.center,
                    circle.radius,
                    36u8,
                    &(0u8, 130u8, 130u8),
                )))?;
            }
            Shape2D::Polygon(_polygon) => {
                todo!()
            }
        }
    }

    for (y_index, row) in grid.map.iter().enumerate() {
        for (x_index, value) in row.iter().enumerate() {
            if value > &0u8 {
                let pos = calculate_position(
                    &Index2D(x_index as i32, y_index as i32),
                    &bottom_left_pos,
                    resolution,
                );
                let error = resolution / 2.0;
                let top_left = (pos.0 - error, pos.1 + error);
                let bottom_right = (pos.0 + error, pos.1 - error);
                chart.draw_series(std::iter::once(Rectangle::new(
                    [top_left, bottom_right],
                    &RED,
                )))?;
            }
        }
    }

    Ok(())
}