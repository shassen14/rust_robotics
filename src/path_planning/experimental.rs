use nalgebra::{self as na};
use num_traits::PrimInt;
use std::{cmp::Ordering, collections::HashMap, fmt::Debug};

// Dijkstra
// Rust copy from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/Dijkstra/dijkstra.py

#[derive(Debug, Clone, Copy)]
pub struct Node2 {
    pub x: i32,
    pub y: i32,
    pub parent_index: i32,
    pub cost: usize,
}

// TODO: what is the point of this? No sense of ordering for hashmaps. why???
impl Ord for Node2 {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| self.parent_index.cmp(&other.parent_index))
        // if self.cost < other.cost {
        //     return Ordering::Less;
        // } else if self.cost > other.cost {
        //     return Ordering::Greater;
        // }
        // Ordering::Equal
    }
}

impl PartialOrd for Node2 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Node2 {
    fn eq(&self, other: &Self) -> bool {
        return (self.x, self.y) == (other.x, other.y);
    }
}

impl Eq for Node2 {}

pub struct Motion2 {
    pub dx: i32,
    pub dy: i32,
    pub cost: usize,
}

const MODEL: [Motion2; 8] = [
    Motion2 {
        dx: 1,
        dy: 0,
        cost: 100,
    },
    Motion2 {
        dx: -1,
        dy: 0,
        cost: 100,
    },
    Motion2 {
        dx: 0,
        dy: 1,
        cost: 100,
    },
    Motion2 {
        dx: 0,
        dy: -1,
        cost: 100,
    },
    Motion2 {
        dx: 1,
        dy: 1,
        cost: 141,
    },
    Motion2 {
        dx: -1,
        dy: 1,
        cost: 141,
    },
    Motion2 {
        dx: 1,
        dy: -1,
        cost: 141,
    },
    Motion2 {
        dx: -1,
        dy: -1,
        cost: 141,
    },
];

pub struct Dijkstra2 {
    // pub map: Vec<Vec<bool>>,
    pub resolution: f64,
    pub x_bounds: (f64, f64),
    pub y_bounds: (f64, f64),
    pub x_length: f64, // map index length
    pub y_length: f64, // map index length
}

impl Dijkstra2 {
    pub fn new(resolution: f64, x_bounds: (f64, f64), y_bounds: (f64, f64)) -> Self {
        Dijkstra2 {
            resolution: resolution,
            x_bounds: x_bounds,
            y_bounds: y_bounds,
            x_length: x_bounds.1 - x_bounds.0,
            y_length: y_bounds.1 - y_bounds.0,
        }
    }

    // index is just 0, 1, 2, 3... f64 to make it easier to math operations
    pub fn calculate_position(
        &self,
        index: &na::SVector<f64, 2>,
        min_position: &na::SVector<f64, 2>,
    ) -> na::SVector<f64, 2> {
        return self.resolution * index + min_position;
    }

    pub fn calculate_position_1d(&self, index: f64, min_position_1d: f64) -> f64 {
        return self.resolution * index + min_position_1d;
    }

    pub fn calculate_index_2d(
        &self,
        position: &na::SVector<f64, 2>,
        min_position: &na::SVector<f64, 2>,
    ) -> na::SVector<i32, 2> {
        let index_approx = (position - min_position) / self.resolution;
        return na::SVector::<i32, 2>::new(
            index_approx.x.round() as i32,
            index_approx.y.round() as i32,
        );
    }

    pub fn calculate_index(&self, node: &Node2) -> i32 {
        let index_approx = (node.y as f64 - self.y_bounds.0) * self.x_length as f64
            + (node.x as f64 - self.x_bounds.0);
        index_approx.round() as i32
    }

    pub fn verify_node(&self, node: &Node2) -> bool {
        let pos_x = self.calculate_position_1d(node.x as f64, self.x_bounds.0);
        let pos_y = self.calculate_position_1d(node.y as f64, self.y_bounds.0);

        if pos_x < self.x_bounds.0 || pos_x > self.x_bounds.1 {
            return false;
        }

        if pos_y < self.y_bounds.0 || pos_y > self.y_bounds.1 {
            return false;
        }

        // TODO: check map
        true
    }

    pub fn calculate_final_path(
        &self,
        goal_node: &Node2,
        closed_set: &HashMap<i32, Node2>,
    ) -> Vec<(f64, f64)> {
        let mut path: Vec<(f64, f64)> = vec![(
            self.calculate_position_1d(goal_node.x as f64, self.x_bounds.0),
            self.calculate_position_1d(goal_node.y as f64, self.y_bounds.0),
        )];
        let mut parent_index = goal_node.parent_index;

        while parent_index != -1 {
            let node = *closed_set.get(&parent_index).unwrap();

            path.push((
                self.calculate_position_1d(node.x as f64, self.x_bounds.0),
                self.calculate_position_1d(node.y as f64, self.y_bounds.0),
            ));
            parent_index = node.parent_index;
        }
        path
    }

    pub fn plan(
        &self,
        start_position: &na::SVector<f64, 2>,
        goal_position: &na::SVector<f64, 2>,
    ) -> Vec<(f64, f64)> {
        let min_position = na::SVector::<f64, 2>::new(self.x_bounds.0, self.y_bounds.0);
        let start_index = self.calculate_index_2d(start_position, &min_position);
        let goal_index = self.calculate_index_2d(goal_position, &min_position);

        // vector matrix indices for start and goal node
        let start = Node2 {
            x: start_index.x,
            y: start_index.y,
            cost: 0,
            parent_index: -1,
        };

        let mut goal = Node2 {
            x: goal_index.x,
            y: goal_index.y,
            cost: 0,
            parent_index: -1,
        };

        // TODO: make faster
        let mut open_set: HashMap<i32, Node2> = HashMap::new();
        let mut closed_set: HashMap<i32, Node2> = HashMap::new();
        open_set.insert(self.calculate_index(&start), start);

        // TODO: figure out how to not do while true
        loop {
            println!("Open Set: {:?}", open_set);
            // TODO: unsafe, outputs option
            // TODO: slow. different way than .iter().min(). different data structure? min heap?

            if open_set.is_empty() {
                println!("yooooooo no goal found");
                break;
            }

            // let min_cost_id = *open_set.iter().min().unwrap().0;
            let min_cost_id = *open_set.iter().max_by(|a, b| a.1.cmp(&b.1)).unwrap().0;
            // let min_cost_id = *open_set.iter().next().unwrap().0;

            // TODO: this clones/copies
            let current = *open_set.get(&min_cost_id).unwrap();

            if current.x == goal.x {
                println!("Min Cost ID: {:?}", min_cost_id);
                println!("Current: {:?}", current);
                println!(
                    "Goal: {:?}, goal index: {}",
                    goal,
                    self.calculate_index(&goal)
                );
            }

            if current.x == goal.x && current.y == goal.y {
                println!("Found goal");
                goal.parent_index = current.parent_index;
                goal.cost = current.cost;
                break;
            }

            // TODO: this is ugly
            open_set.remove(&min_cost_id);
            closed_set.insert(min_cost_id, current);

            for motion in MODEL {
                let node = Node2 {
                    x: current.x + motion.dx,
                    y: current.y + motion.dy,
                    cost: current.cost + motion.cost,
                    parent_index: min_cost_id,
                };
                let n_id = self.calculate_index(&node);

                if closed_set.contains_key(&n_id) {
                    // println!("Contains key!!!",);
                    continue;
                }

                if !self.verify_node(&node) {
                    // println!("Node inside!!!",);
                    continue;
                }

                if !open_set.contains_key(&n_id) {
                    open_set.insert(n_id, node);
                } else {
                    if open_set.get(&n_id).unwrap().cost >= node.cost {
                        // This path is the best until now. record it!
                        // this will change n_id key with the value node
                        open_set.insert(n_id, node);
                    }
                }
            }
        }
        println!("End Open Set: {:?}", open_set);
        println!("End Close Set: {:?}", closed_set);
        self.calculate_final_path(&goal, &closed_set)
    }
}

///////////////////////////////////
// redo
///////////////////////////////////

// #[derive(Debug, Clone)]
// struct Node<const N: usize> {
//     index: na::SVector<i32, N>,
//     parent_index: na::SVector<i32, N>,
//     cost: usize,
// }

// impl<const N: usize> Ord for Node<N> {
//     fn cmp(&self, other: &Self) -> Ordering {
//         other
//             .cost
//             .cmp(&self.cost)
//             .then_with(|| self.parent_index.iter().cmp(&other.parent_index))
//     }
// }

// impl<const N: usize> PartialOrd for Node<N> {
//     fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
//         Some(self.cmp(other))
//     }
// }

// impl<const N: usize> PartialEq for Node<N> {
//     fn eq(&self, other: &Self) -> bool {
//         return self.index == other.index;
//     }
// }

// impl<const N: usize> Eq for Node<N> {}

use rustc_hash::FxHashMap;

#[derive(Debug, Hash)]
pub struct IndexKey<T: PrimInt, const N: usize>(pub na::SVector<T, N>)
where
    T: Debug + 'static;
#[derive(Debug, Clone, Copy)]
pub struct Node<T: PrimInt, const N: usize> {
    pub index: na::SVector<T, N>,
    pub parent_index: na::SVector<T, N>,
    pub cost: usize,
}

impl<T: PrimInt, const N: usize> Ord for Node<T, N> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
        // .then_with(|| self.index.iter().cmp(&other.index.iter()))
    }
}

impl<T: PrimInt, const N: usize> PartialOrd for Node<T, N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: PrimInt, const N: usize> PartialEq for Node<T, N> {
    fn eq(&self, other: &Self) -> bool {
        return self.index == other.index;
    }
}

impl<T: PrimInt, const N: usize> Eq for Node<T, N> {}

pub struct Motion<T: PrimInt, const N: usize> {
    pub change: na::SVector<T, N>,
    pub cost: usize,
}

const MODEL2: [Motion<i32, 2>; 8] = [
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(1, 0),
        cost: 100,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(0, 1),
        cost: 100,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(-1, 0),
        cost: 100,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(0, -1),
        cost: 100,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(1, 1),
        cost: 141,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(-1, 1),
        cost: 141,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(1, -1),
        cost: 141,
    },
    Motion::<i32, 2> {
        change: na::SVector::<i32, 2>::new(-1, -1),
        cost: 141,
    },
];

pub struct D2 {
    resolution: f64,
    x_bounds: [f64; 2],
    y_bounds: [f64; 2],
}
impl D2 {
    pub fn new(resolution: f64, x_bounds: [f64; 2], y_bounds: [f64; 2]) -> Self {
        D2 {
            resolution: resolution,
            x_bounds: x_bounds,
            y_bounds: y_bounds,
        }
    }

    pub fn calculate_position_1d(&self, index: f64, min_position_1d: f64) -> f64 {
        return self.resolution * index + min_position_1d;
    }

    pub fn calculate_position(
        &self,
        index: &na::SVector<i32, 2>,
        min_position: &na::SVector<f64, 2>,
    ) -> na::SVector<f64, 2> {
        return self.resolution * na::SVector::<f64, 2>::new(index.x as f64, index.y as f64)
            + min_position;
    }

    pub fn calculate_index_2d(
        &self,
        position: &na::SVector<f64, 2>,
        min_position: &na::SVector<f64, 2>,
    ) -> na::SVector<i32, 2> {
        let index_approx = (position - min_position) / self.resolution;
        return na::SVector::<i32, 2>::new(
            index_approx.x.round() as i32,
            index_approx.y.round() as i32,
        );
    }

    pub fn verify_node(&self, node: &Node<i32, 2>) -> bool {
        let pos = self.calculate_position(
            &node.index,
            &na::SVector::<f64, 2>::new(self.x_bounds[0], self.y_bounds[0]),
        );

        if pos.x < self.x_bounds[0] || pos.x > self.x_bounds[1] {
            return false;
        }

        if pos.y < self.y_bounds[0] || pos.y > self.y_bounds[1] {
            return false;
        }

        // TODO: check map
        true
    }

    pub fn calculate_final_path(
        &self,
        start_node: &Node<i32, 2>,
        goal_node: &Node<i32, 2>,
        closed_set: &FxHashMap<na::SVector<i32, 2>, Node<i32, 2>>,
    ) -> Vec<(f64, f64)> {
        let mut path: Vec<(f64, f64)> = vec![(
            self.calculate_position_1d(goal_node.index.x as f64, self.x_bounds[0]),
            self.calculate_position_1d(goal_node.index.y as f64, self.y_bounds[0]),
        )];
        let mut current_index = goal_node.index;
        let mut parent_index = goal_node.parent_index;

        while current_index != start_node.index {
            let node = *closed_set.get(&parent_index).unwrap();
            current_index = node.index;

            path.push((
                self.calculate_position_1d(node.index.x as f64, self.x_bounds[0]),
                self.calculate_position_1d(node.index.y as f64, self.y_bounds[0]),
            ));
            parent_index = node.parent_index;
        }
        path.into_iter().rev().collect()
    }

    pub fn plan(
        &self,
        start_position: &na::SVector<f64, 2>,
        goal_position: &na::SVector<f64, 2>,
    ) -> Vec<(f64, f64)> {
        let min_position = na::SVector::<f64, 2>::new(self.x_bounds[0], self.y_bounds[0]);
        let start_index = self.calculate_index_2d(start_position, &min_position);
        let goal_index = self.calculate_index_2d(goal_position, &min_position);

        // vector matrix indices for start and goal node
        let start = Node::<i32, 2> {
            index: na::SVector::<i32, 2>::new(start_index.x, start_index.y),
            parent_index: na::SVector::<i32, 2>::new(-1, -1),
            cost: 0,
        };

        let mut goal = Node::<i32, 2> {
            index: na::SVector::<i32, 2>::new(goal_index.x, goal_index.y),
            parent_index: na::SVector::<i32, 2>::new(goal_index.x, goal_index.y),
            cost: 0,
        };

        // TODO: make faster
        let mut open_set: FxHashMap<na::SVector<i32, 2>, Node<i32, 2>> = FxHashMap::default();
        let mut closed_set: FxHashMap<na::SVector<i32, 2>, Node<i32, 2>> = FxHashMap::default();
        open_set.insert(start.index, start);

        while !open_set.is_empty() {
            // TODO: unsafe, outputs option
            // TODO: slow. different way than .iter().min(). different data structure? min heap?
            // let min_cost_id = *open_set.iter().min().unwrap().0;
            // TODO need to figure out a different way to find this index
            let min_cost_id = *open_set.iter().max_by(|a, b| a.1.cmp(&b.1)).unwrap().0;

            // TODO: this clones/copies
            let current = *open_set.get(&min_cost_id).unwrap();

            if current.index == goal.index {
                println!("Found goal");
                goal.parent_index = current.parent_index;
                goal.cost = current.cost;
                break;
            }

            // TODO: this is ugly
            open_set.remove(&min_cost_id);
            closed_set.insert(min_cost_id, current);

            for motion in MODEL2 {
                let node = Node::<i32, 2> {
                    index: current.index + motion.change,
                    parent_index: min_cost_id,
                    cost: current.cost + motion.cost,
                };

                if closed_set.contains_key(&node.index) {
                    // println!("Contains key!!!",);
                    continue;
                }

                if !self.verify_node(&node) {
                    // println!("Node inside!!!",);
                    continue;
                }

                if !open_set.contains_key(&node.index) {
                    open_set.insert(node.index, node);
                } else {
                    if open_set.get(&node.index).unwrap().cost >= node.cost {
                        // This path is the best until now. record it!
                        // this will change n_id key with the value node
                        open_set.insert(node.index, node);
                    }
                }
            }
        }
        self.calculate_final_path(&start, &goal, &closed_set)
    }
}
