use rust_robotics::path_planning::experimental;
use rust_robotics::utils::files;
use rust_robotics::utils::plot2;

// Plotters
use nalgebra as na;
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::RGBPixel;
use plotters_bitmap::BitMapBackend;
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    // Read command line arguments
    let args: Vec<String> = env::args().collect();

    // Obtain config_path from command line
    // TODO: make a class for this to streamline this and send helpful error messages
    let animate_cfg_path = &args[1] as &str;

    // Obtain plot config params given the file
    let plot_config: plot2::Config = files::read_config(animate_cfg_path);

    let chart_params: plot2::ChartParams = plot_config.chart_params;
    let window_params: plot2::WindowParams = plot_config.window_params;

    let root = BitMapBackend::<RGBPixel>::new(
        "bob.png",
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

    let object = experimental::Dijkstra2 {
        resolution: 0.5,
        x_bounds: (chart_params.x_range[0], chart_params.x_range[1]),
        y_bounds: (chart_params.y_range[0], chart_params.y_range[1]),
        x_length: chart_params.x_range[1] - chart_params.x_range[0],
        y_length: chart_params.y_range[1] - chart_params.y_range[0],
    };

    let dijkstra_path = object.plan(
        &na::SVector::<f64, 2>::new(0.0, 0.0),
        &na::SVector::<f64, 2>::new(1.0, 4.0),
    );
    // let data = [(-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5), (-0.5, -0.5)];

    chart.draw_series(LineSeries::new(dijkstra_path, &YELLOW))?;
    // chart.draw_series(std::iter::once(Rectangle::new(
    //     [(0.5, 0.5), (0.75, 0.75)],
    //     &WHITE,
    // )))?;

    // let path = PathElement::new(
    //     [
    //         (-0.9, 0.9),
    //         (-0.8, 0.8),
    //         (-0.7, 0.85),
    //         (-0.8, 0.9),
    //         (-0.9, 0.9),
    //     ],
    //     &PURPLE,
    // );

    // chart.draw_series(std::iter::once(path))?;

    Ok(())
}
