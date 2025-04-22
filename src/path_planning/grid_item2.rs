use num_traits::{Float, PrimInt};

use crate::path_planning::base::SearchSpace;
use std::cmp::Ordering;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Index2D {
    pub row: usize,
    pub col: usize,
}

impl Ord for Index2D {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.row == other.row {
            self.col.cmp(&other.col)
        } else {
            self.row.cmp(&other.row)
        }
    }
}

impl PartialOrd for Index2D {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position2D<T> {
    pub x: T,
    pub y: T,
}

#[derive(Debug, Clone)]
pub struct GridSpec<T> {
    pub origin: Position2D<T>, // World coordinates of grid[0][0]
    pub resolution: T,         // Size of a grid cell in world units
    pub rows: usize,
    pub cols: usize,
}

impl<T> GridSpec<T>
where
    T: num_traits::Float + std::ops::Add<Output = T> + Copy,
{
    pub fn position_to_index(&self, pos: Position2D<T>) -> Option<Index2D> {
        let dx = pos.x - self.origin.x;
        let dy = pos.y - self.origin.y;

        if dx < T::zero() || dy < T::zero() {
            return None;
        }

        let col = (dx / self.resolution).to_usize()?;
        let row = (dy / self.resolution).to_usize()?;

        if row < self.rows && col < self.cols {
            Some(Index2D { row, col })
        } else {
            None
        }
    }

    pub fn index_to_position(&self, idx: Index2D) -> Position2D<T> {
        let x = self.origin.x + T::from(idx.col).unwrap() * self.resolution;
        let y = self.origin.y + T::from(idx.row).unwrap() * self.resolution;
        Position2D { x, y }
    }
}
// GridMap representing the grid with costs associated to each grid cell
pub struct GridMap<T, R>
where
    T: Float + Copy,   // T is float for position and resolution
    R: PrimInt + Copy, // R is PrimInt for cost (integer)
{
    pub cost_map: Vec<Vec<R>>, // Now cost_map is of integer type (e.g., u32)
    pub grid_spec: GridSpec<T>,
}

impl<T, R> GridMap<T, R>
where
    T: Float + Copy,   // T is float for position and resolution
    R: PrimInt + Copy, // R is PrimInt for cost (integer)
{
    // Constructor for GridMap
    pub fn new(rows: usize, cols: usize, resolution: T, default_cost: R) -> Self {
        let cost_map = vec![vec![default_cost; cols]; rows];
        let grid_spec = GridSpec {
            rows,
            cols,
            origin: Position2D {
                x: T::zero(),
                y: T::zero(),
            },
            resolution,
        };
        Self {
            cost_map,
            grid_spec,
        }
    }

    // Check if a grid index is valid
    pub fn is_valid(&self, idx: Index2D) -> bool {
        idx.row < self.grid_spec.rows && idx.col < self.grid_spec.cols
    }

    // Get cost at a specific index
    pub fn cost(&self, idx: Index2D) -> Option<R> {
        if self.is_valid(idx) {
            Some(self.cost_map[idx.row][idx.col])
        } else {
            None
        }
    }

    // Get neighbors with their cost
    pub fn neighbors_with_cost(&self, idx: Index2D) -> Vec<(Index2D, R)> {
        let deltas = [(-1isize, 0), (1, 0), (0, -1), (0, 1)];
        let mut result = Vec::new();

        for (dr, dc) in deltas.iter() {
            let new_row = idx.row as isize + dr;
            let new_col = idx.col as isize + dc;

            if new_row >= 0 && new_col >= 0 {
                let new_idx = Index2D {
                    row: new_row as usize,
                    col: new_col as usize,
                };
                if let Some(cost) = self.cost(new_idx) {
                    result.push((new_idx, cost));
                }
            }
        }

        result
    }

    pub fn convert_path_to_world(&self, path: &[Index2D]) -> Vec<Position2D<T>> {
        path.iter()
            .map(|idx| self.grid_spec.index_to_position(*idx))
            .collect()
    }
}

impl<T, R> SearchSpace<Index2D, R> for GridMap<T, R>
where
    T: Float + Copy,   // T is float for position and resolution
    R: PrimInt + Copy, // R is PrimInt for cost (integer)
{
    fn neighbors(&self, node: &Index2D) -> Vec<(Index2D, R)> {
        self.neighbors_with_cost(*node)
    }
}
