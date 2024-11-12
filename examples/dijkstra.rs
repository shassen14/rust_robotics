use rust_robotics::path_planning::dijkstra;
use rust_robotics::utils::files;
use rust_robotics::utils::geometry;
use rust_robotics::utils::geometry::CircleS;
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

fn calculate_index(position: &Position2D, min_position: &Position2D, resolution: f64) -> Index2D {
    Index2D(
        ((position.0 - min_position.0) / resolution).round() as i32,
        ((position.1 - min_position.1) / resolution).round() as i32,
    )
}

fn calculate_cost(index: &Index2D) -> usize {
    let index_f: (f64, f64) = (index.0 as f64, index.1 as f64);
    (f64::sqrt(index_f.0 * index_f.0 + index_f.1 * index_f.1) * 100.0).round() as usize
}

impl Index2D {
    // first index is lower left, 2nd index is top right
    fn verify_node(&self, map: &Vec<Vec<u8>>, index_bounds: [Index2D; 2]) -> bool {
        if self.0 < index_bounds[0].0 || self.0 > index_bounds[1].0 {
            return false;
        }

        if self.1 < index_bounds[0].1 || self.1 > index_bounds[1].1 {
            return false;
        }

        if map[self.1 as usize][self.0 as usize] > 0 {
            return false;
        }

        true
    }

    fn populate_children(
        &self,
        map: &Vec<Vec<u8>>,
        index_bounds: &[Index2D; 2],
        model: &Vec<Index2D>,
    ) -> Vec<(Index2D, usize)> {
        // item is (position, cost)
        let mut children: Vec<(Index2D, usize)> = Vec::default();

        for motion in model {
            let node = Index2D(self.0 + motion.0, self.1 + motion.1);

            if node.verify_node(map, *index_bounds) {
                children.push((node, calculate_cost(&motion)));
            }
        }

        children
    }
}

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

    let resolution = 1.0;
    let bottom_left_pos = Position2D(chart_params.x_range[0], chart_params.y_range[0]);
    let top_right_pos = Position2D(chart_params.x_range[1], chart_params.y_range[1]);
    let start_pos = Position2D(-30.0, -65.0);
    let goal_pos = Position2D(0.0, 70.0);

    let bottom_left_index = calculate_index(&bottom_left_pos, &bottom_left_pos, resolution);
    let top_right_index = calculate_index(&top_right_pos, &bottom_left_pos, resolution);
    let start_index = calculate_index(&start_pos, &bottom_left_pos, resolution);
    let goal_index = calculate_index(&goal_pos, &bottom_left_pos, resolution);

    let model: Vec<Index2D> = vec![
        Index2D(1, 0),
        Index2D(0, 1),
        Index2D(-1, 0),
        Index2D(0, -1),
        Index2D(1, 1),
        Index2D(-1, 1),
        Index2D(1, -1),
        Index2D(-1, -1),
    ];

    let obstacles = vec![
        (
            geometry::Shape2D::Circle(CircleS::new((0.0, 0.0), 1.0)),
            1u8,
        ),
        (
            geometry::Shape2D::Circle(CircleS::new((5.0, 0.0), 2.0)),
            1u8,
        ),
        (
            geometry::Shape2D::Circle(CircleS::new((20.0, 20.0), 5.0)),
            1u8,
        ),
        (
            geometry::Shape2D::Circle(CircleS::new((0.0, 50.0), 10.0)),
            1u8,
        ),
        (
            geometry::Shape2D::Circle(CircleS::new((-30.0, -50.0), 10.0)),
            1u8,
        ),
        (
            geometry::Shape2D::Circle(CircleS::new((-20.0, 0.0), 15.0)),
            1u8,
        ),
    ];

    let grid = map_generator::GridMap2D::new(
        &obstacles,
        &chart_params.x_range,
        &chart_params.y_range,
        resolution,
    );

    println!(
        "Start Position: {:?} \tStart Index: {:?}",
        start_pos, start_index
    );

    println!(
        "Goal Position: {:?} \tGoal Index: {:?}",
        goal_pos, goal_index
    );

    let dijkstra_path = dijkstra::plan(&start_index, &goal_index, &mut |p| {
        p.populate_children(&grid.map, &[bottom_left_index, top_right_index], &model)
    });

    chart.draw_series(std::iter::once(Cross::new(
        (start_pos.0, start_pos.1),
        5,
        &RED,
    )))?;
    chart.draw_series(std::iter::once(Cross::new(
        (goal_pos.0, goal_pos.1),
        5,
        &RED,
    )))?;

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
            Shape2D::Polygon(polygon) => {}
        }
    }

    let pos_path: Vec<(f64, f64)> = dijkstra_path
        .expect("No path found")
        .into_iter()
        .map(|i| {
            let pos = calculate_position(&i, &bottom_left_pos, resolution);
            (pos.0, pos.1)
        })
        .collect();

    println!("d path: {:?}", pos_path);

    chart.draw_series(LineSeries::new(pos_path, &YELLOW))?;

    Ok(())
}
