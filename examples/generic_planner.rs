use num_traits::Float;
use rust_robotics::path_planning::base::PathPlanner;
use rust_robotics::path_planning::dijkstras2::DijkstraPlanner;
use rust_robotics::path_planning::grid_item2::{GridMap, GridSpec, Index2D, Position2D}; // <-- THIS IS CRUCIAL

pub fn convert_path_to_world<T>(grid_spec: &GridSpec<T>, path: &[Index2D]) -> Vec<Position2D<T>>
where
    T: Float,
{
    path.iter()
        .map(|idx| grid_spec.index_to_position(*idx))
        .collect()
}

fn main() {
    let mut cost_map = vec![vec![1; 6]; 6];
    cost_map[1][1] = 100;

    let grid_spec = GridSpec {
        origin: Position2D { x: -1.0, y: -1.0 },
        resolution: 0.5,
        rows: 6,
        cols: 6,
    };

    let grid_spec_clone = grid_spec.clone();

    let map = GridMap {
        cost_map,
        grid_spec,
    };

    let mut planner = DijkstraPlanner::new(map);
    let start = Index2D { row: 0, col: 0 };
    let goal = Index2D { row: 5, col: 5 };

    planner.set_start_goal(start, goal);
    let path = planner.plan();

    if let Some(path) = path {
        println!("Path found:");
        let world_path: Vec<Position2D<f64>> = convert_path_to_world(&grid_spec_clone, &path);
        for step in path {
            println!(" -> {:?}", step);
        }
        for step in world_path {
            println!(" -> {:?}", step);
        }
    } else {
        println!("No path found");
    }
}
