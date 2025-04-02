use num_traits::{AsPrimitive, Float, PrimInt};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Index2D<T>(pub T, pub T);

impl<T> Index2D<T>
where
    T: PrimInt + AsPrimitive<usize>,
{
    pub fn populate_neighbors(
        &self,
        map: &Vec<Vec<u8>>,
        index_bounds: &[Index2D<T>; 2],
        motion_model: &Vec<Index2D<T>>,
    ) -> Vec<(Index2D<T>, usize)>
    where
        T: AsPrimitive<f32>,
    {
        let mut neighbors: Vec<(Index2D<T>, usize)> = Vec::new();

        for motion in motion_model {
            let node = Index2D(self.0 + motion.0, self.1 + motion.1);

            if node.verify_node(map, *index_bounds) {
                neighbors.push((node, calculate_cost(motion)));
            }
        }

        neighbors
    }

    fn verify_node(&self, map: &Vec<Vec<u8>>, index_bounds: [Index2D<T>; 2]) -> bool {
        // TODO: assuming valid input
        // assert
        // index_bounds -> [(x_min, y_min), (x_max, y_max)]
        if self.0 < index_bounds[0].0 || self.0 > index_bounds[1].0 {
            return false;
        }

        if self.1 < index_bounds[0].1 || self.1 > index_bounds[1].1 {
            return false;
        }

        // if an obstacle, only free space is 0
        // TODO: better check
        if map[self.1.as_()][self.0.as_()] > 0 {
            return false;
        }

        return true;
    }
}

#[derive(Clone, Debug)]
pub struct Position2D<U>(pub U, pub U)
where
    U: Float;

pub fn calculate_position<T, U>(
    index: &Index2D<T>,
    min_position: &Position2D<U>,
    resolution: U,
) -> Position2D<U>
where
    T: PrimInt + AsPrimitive<U>,
    U: Float + 'static,
{
    Position2D(
        resolution * index.0.as_() + min_position.0,
        resolution * index.1.as_() + min_position.1,
    )
}

pub fn calculate_index<T, U>(
    position: &Position2D<U>,
    min_position: &Position2D<U>,
    resolution: U,
) -> Index2D<T>
where
    T: PrimInt + 'static,
    U: Float + AsPrimitive<T>,
{
    Index2D(
        ((position.0 - min_position.0) / resolution).round().as_(),
        ((position.1 - min_position.1) / resolution).round().as_(),
    )
}

// TODO ugly... better cost?
// TODO change return to be generic?
// Random * 100 aka take into account 2 decimal places
pub fn calculate_cost<T>(index: &Index2D<T>) -> usize
where
    T: PrimInt + AsPrimitive<f32>,
{
    let index_f = (index.0.as_(), index.1.as_());

    f32::sqrt((index_f.0 * index_f.0 + index_f.1 * index_f.1) * 100.0).round() as usize
}

// |node| {reach_goal(node....) -> bool}
pub fn reach_goal<T>(goal_index: &Index2D<T>, current_node_index: &Index2D<T>) -> bool
where
    T: PrimInt,
{
    // converting this index to a position
    // if the goal_pos is in an obstacle, then find the difference between the center and the goal_pos
    // Then use the obstacle radius with this difference to inflate the goal position

    let mut is_finish = false;

    if current_node_index == goal_index {
        is_finish = true;
    }

    is_finish
}

// TODO: yeaaah this needs to be better, too many cass
pub fn move_invalid_index<T>(
    start_index: &mut Index2D<T>,
    goal_index: &mut Index2D<T>,
    grid_map: &Vec<Vec<u8>>,
) where
    T: PrimInt + AsPrimitive<f32> + AsPrimitive<usize>,
    f32: AsPrimitive<T>,
    usize: AsPrimitive<T>,
{
    // Calculate slope from the start the goal
    let dif_x = start_index.0 - goal_index.0;
    let dif_y = start_index.1 - goal_index.1;
    let dif_x_f32: f32 = dif_x.as_();
    let dif_y_f32: f32 = dif_y.as_();
    let magnitude = f32::sqrt((dif_x * dif_x + dif_y * dif_y).as_());

    let slope_angle = f32::atan2(dif_y.as_(), dif_x.as_());
    let mut start_row: usize = start_index.1.as_();
    let mut start_col: usize = start_index.0.as_();

    // TODO: check math with x, y vs row, col
    // TODO issue is usize, there are not negatives with slope part of the equation
    while grid_map[start_row][start_col] != 0 {
        let new_x = start_col as i32 + (slope_angle * dif_x_f32 / magnitude).ceil() as i32;
        let new_y = start_row as i32 + (slope_angle * dif_y_f32 / magnitude).ceil() as i32;

        start_col = new_x as usize;
        start_row = new_y as usize;
    }

    let mut goal_row: usize = goal_index.1.as_();
    let mut goal_col: usize = goal_index.0.as_();

    while grid_map[goal_row][goal_col] != 0 {
        let new_x = goal_col as i32 - (slope_angle * dif_x_f32 / magnitude).ceil() as i32;
        let new_y = goal_row as i32 - (slope_angle * dif_y_f32 / magnitude).ceil() as i32;

        goal_col = new_x as usize;
        goal_row = new_y as usize;
    }

    start_index.0 = start_col.as_();
    start_index.1 = start_row.as_();

    goal_index.0 = goal_col.as_();
    goal_index.1 = goal_row.as_();
}
