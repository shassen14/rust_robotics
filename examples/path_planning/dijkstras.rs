// rust robotics
use rust_robotics::path_planning::dijkstras;
use rust_robotics::utils::defs;
use rust_robotics::utils::files;
use rust_robotics::utils::geometry::Shape2D;
use rust_robotics::utils::map_generator;
use rust_robotics::utils::plot2;

// 3rd party or std
use clap::Parser;
use core::f64;
use plotters::prelude::*;
use plotters_bitmap::{bitmap_pixel::BGRXPixel, BitMapBackend};
use std::borrow::{Borrow, BorrowMut};
use std::collections::VecDeque;
use std::env;
use std::error::Error;
use std::time::SystemTime;

/// A program to generate a map given the plot size and also obstacles to map within the plot
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// File that has all the obstacles and animation params within
    #[arg(short, long)]
    dijkstras_config: String,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Index2D(i32, i32);

impl Index2D {
    fn verify_node(&self, map: &Vec<Vec<u8>>, index_bounds: [Index2D; 2]) -> bool {
        // TODO: assuming valid input
        // assert
        // index_bounds -> [(x_min, y_min), (x_max, y_max)]
        if self.0 < index_bounds[0].0 || self.0 >= index_bounds[1].0 {
            return false;
        }

        if self.1 < index_bounds[0].1 || self.1 >= index_bounds[1].1 {
            return false;
        }

        // if an obstacle, only free space is 0
        // TODO: better check
        if map[self.1 as usize][self.0 as usize] > 0 {
            return false;
        }

        return true;
    }

    fn populate_neighbors(
        &self,
        map: &Vec<Vec<u8>>,
        index_bounds: &[Index2D; 2],
        motion_model: &Vec<Index2D>,
    ) -> Vec<(Index2D, usize)> {
        let mut neighbors: Vec<(Index2D, usize)> = Vec::new();

        for motion in motion_model {
            let node = Index2D(self.0 + motion.0, self.1 + motion.1);

            if node.verify_node(map, *index_bounds) {
                neighbors.push((node, calculate_cost(motion)));
            }
        }

        neighbors
    }

    // |node| node.populate_neighbors()
}

#[derive(Clone, Debug)]
struct Position2D(f64, f64);

fn calculate_position(index: &Index2D, min_position: &Position2D, resolution: f64) -> Position2D {
    Position2D(
        resolution * index.0 as f64 + min_position.0,
        resolution * index.1 as f64 + min_position.1,
    )
}

fn calculate_index(position: &Position2D, min_position: &Position2D, resolution: f64) -> Index2D {
    Index2D(
        ((position.0 - min_position.0) / resolution).round() as i32,
        ((position.1 - min_position.1) / resolution).round() as i32,
    )
}

// TODO ugly... better cost?
fn calculate_cost(index: &Index2D) -> usize {
    let index_f = (index.0 as f32, index.1 as f32);

    f32::sqrt((index_f.0 * index_f.0 + index_f.1 * index_f.1) * 100.0).round() as usize
}

fn main() -> Result<(), Box<dyn Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    // Read command line arguments
    let args: Args = Args::parse();

    // Obtain plot config params given the file
    let plot_config: plot2::Config = files::read_toml(&args.dijkstras_config);
    let map_params: map_generator::MapGeneratorParams<f64, u8> =
        files::read_toml(&args.dijkstras_config);

    // Acquire animation params
    let chart_params: plot2::ChartParams = plot_config.chart_params;
    let window_params: plot2::WindowParams = plot_config.window_params;
    let animation_params: plot2::AnimationParams = plot_config.animation_params;

    let mut buf = defs::BufferWrapper(vec![0u32; window_params.width * window_params.height]);

    let mut window = plot2::create_window(&window_params)?;

    let cs = plot2::create_2d_chartstate(buf.borrow_mut(), &window_params, &chart_params);

    let start_time = SystemTime::now();
    // let sample_step: f64 = 1.0 / animation_params.sample_rate;
    let frame_step: f64 = 1.0 / animation_params.frame_rate;

    // mapping
    // TODO: put these in configs?
    let resolution = 0.5;
    let start_pos = Position2D(-60., -60.);
    let goal_pos = Position2D(20.0, 40.0);
    let bottom_left_pos = Position2D(chart_params.x_range[0], chart_params.y_range[0]);
    let top_right_pos = Position2D(chart_params.x_range[1], chart_params.y_range[1]);

    // calculate indices
    let start_index = calculate_index(&start_pos, &bottom_left_pos, resolution);
    let goal_index = calculate_index(&goal_pos, &bottom_left_pos, resolution);
    let bottom_left_index = calculate_index(&bottom_left_pos, &bottom_left_pos, resolution);
    let top_right_index = calculate_index(&top_right_pos, &bottom_left_pos, resolution);

    let mut mouse_chart_start_pos: (f64, f64) = (start_pos.0, start_pos.1);
    let mut mouse_chart_goal_pos: (f64, f64) = (goal_pos.0, goal_pos.1);

    // Creating the obstacle map
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

    // Motion model
    // TODO: Maybe have in config?
    let model = vec![
        Index2D(1, 0),
        Index2D(-1, 0),
        Index2D(1, 1),
        Index2D(-1, -1),
        Index2D(-1, 1),
        Index2D(1, -1),
        Index2D(0, 1),
        Index2D(0, -1),
    ];

    // Vector of the index values x, y array indices
    let index_path = dijkstras::plan(&start_index, &goal_index, &mut |node| {
        node.populate_neighbors(&grid.map, &[bottom_left_index, top_right_index], &model)
    });

    let pos_path: Vec<(f64, f64)> = index_path
        .expect("No Path Found")
        .into_iter()
        .map(|i| {
            let pos = calculate_position(&i, &bottom_left_pos, resolution);
            (pos.0, pos.1)
        })
        .collect();

    // time, vector of positions
    let mut data: VecDeque<(f64, Vec<(f64, f64)>)> = VecDeque::new();
    let mut last_flushed = SystemTime::now().duration_since(start_time)?.as_secs_f64();
    data.push_back((last_flushed, pos_path));

    while window.is_open() && !window.is_key_down(minifb::Key::Escape) {
        let epoch = SystemTime::now().duration_since(start_time)?.as_secs_f64();

        let mouse_start_pos = if window.get_mouse_down(minifb::MouseButton::Right) {
            window.get_mouse_pos(minifb::MouseMode::Clamp)
        } else {
            None
        };

        let mouse_goal_pos = if window.get_mouse_down(minifb::MouseButton::Left) {
            window.get_mouse_pos(minifb::MouseMode::Clamp)
        } else {
            None
        };

        // TODO: Repetitive code
        if let Some(coord) = mouse_start_pos {
            mouse_chart_start_pos = plot2::mouse_chart_position(
                (coord.0 as f64, coord.1 as f64),
                &window_params,
                &chart_params,
            );

            let mouse_start_index = calculate_index(
                &Position2D(mouse_chart_start_pos.0, mouse_chart_start_pos.1),
                &bottom_left_pos,
                resolution,
            );

            let mouse_goal_index = calculate_index(
                &Position2D(mouse_chart_goal_pos.0, mouse_chart_goal_pos.1),
                &bottom_left_pos,
                resolution,
            );

            let path: Vec<(f64, f64)> =
                dijkstras::plan(&mouse_start_index, &mouse_goal_index, &mut |node| {
                    node.populate_neighbors(
                        &grid.map,
                        &[bottom_left_index, top_right_index],
                        &model,
                    )
                })
                .expect("No Path Found")
                .into_iter()
                .map(|i| {
                    let pos = calculate_position(&i, &bottom_left_pos, resolution);
                    (pos.0, pos.1)
                })
                .collect();

            data.push_back((epoch, path));
            data.pop_front();
        } else if let Some(coord) = mouse_goal_pos {
            mouse_chart_goal_pos = plot2::mouse_chart_position(
                (coord.0 as f64, coord.1 as f64),
                &window_params,
                &chart_params,
            );

            let mouse_start_index = calculate_index(
                &Position2D(mouse_chart_start_pos.0, mouse_chart_start_pos.1),
                &bottom_left_pos,
                resolution,
            );

            let mouse_goal_index = calculate_index(
                &Position2D(mouse_chart_goal_pos.0, mouse_chart_goal_pos.1),
                &bottom_left_pos,
                resolution,
            );

            let path: Vec<(f64, f64)> =
                dijkstras::plan(&mouse_start_index, &mouse_goal_index, &mut |node| {
                    node.populate_neighbors(
                        &grid.map,
                        &[bottom_left_index, top_right_index],
                        &model,
                    )
                })
                .expect("No Path Found")
                .into_iter()
                .map(|i| {
                    let pos = calculate_position(&i, &bottom_left_pos, resolution);
                    (pos.0, pos.1)
                })
                .collect();

            data.push_back((epoch, path));
            data.pop_front();
        }

        if epoch - last_flushed > frame_step {
            while data.len() > 1 {
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

                // Draw all shapes onto the map
                for (shape, _cost) in &obstacles {
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

                if let Some((_, path)) = data.back() {
                    chart.draw_series(LineSeries::new(path.clone(), &GREEN))?;
                }

                chart.draw_series(std::iter::once(Cross::new(
                    mouse_chart_start_pos,
                    5,
                    &RGBColor(255, 0, 0),
                )))?;

                chart.draw_series(std::iter::once(Cross::new(
                    mouse_chart_goal_pos,
                    5,
                    &RGBColor(0, 0, 255),
                )))?;
            }

            window.update_with_buffer(buf.borrow(), window_params.width, window_params.height)?;
            last_flushed = epoch;
        }
    }

    Ok(())
}

// fn main() {}
