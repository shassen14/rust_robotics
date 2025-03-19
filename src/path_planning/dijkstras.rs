/**
 * dijkstra(start: N, goal: N, children: FnMut(&Node) -> Vec some iterable data struct) -> Optional<list>
 *
 * N -> (f32, f32), (f32, i32), (f64, f64, f64, f64)
 *
 * ////////////////////
 * N = (i32, i32)
 * FN(N) -> Vec<(N, C)>
 *
 * grid: Vec<Vec<int>> costmap;
 *
 * fn children_fn(node: N) {
 *    answer: Vec<(N, C)>
 *    dirs = {{1, 0}, {0, 1}}
 *
 *  
 *
 *    for dir in dirs {
 *    neighbor = (node.first + dir.first, node.second + dir.second)
 *
 *    if (costmap[neighbor.first][neighbor.second] == 0) {
 *       answer.push_back(neighbor, 0);
 *      }
 *    }
 *
 *    answer
 *
 * }
 *
 * How to check if we have seen N node before?
 * Set?
 *
 */
use num_traits::Zero;
use std::collections::HashMap;
use std::hash::Hash;

// Define a structure for the priority queue element
#[derive(Debug, Copy, Clone)]
struct DijkstraItem<N, C> {
    node: N,
    parent: Option<N>,
    cost: C,
}

// // Implement PartialEq for Item (if you want equality checks)
// impl<N, C: PartialEq> PartialEq for DijkstraItem<N, C> {
//     fn eq(&self, other: &Self) -> bool {
//         self.cost == other.cost
//     }
// }

// // Implement Eq for Item
// impl<N, C: Eq> Eq for DijkstraItem<N, C> {}

// // Implement PartialOrd and Ord for Item
// impl<N, C: Ord> PartialOrd for DijkstraItem<N, C> {
//     fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
//         Some(self.cost.cmp(&other.cost)) // Compare by the cost
//     }
// }

// impl<N, C: Ord> Ord for DijkstraItem<N, C> {
//     fn cmp(&self, other: &Self) -> Ordering {
//         other.cost.cmp(&self.cost) // Reverse the order to make it a min-heap
//     }
// }

/// Plans a path from start to goal using the children to expand the nodes
///
///
///
///
pub fn plan<N, C, FN, IT>(start: &N, goal: &N, neighbor_fn: &mut FN) -> Option<Vec<N>>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IT,
    IT: IntoIterator<Item = (N, C)>,
{
    let mut path: Option<Vec<N>> = Option::default();
    let mut is_path_found: bool = false;
    let s = *start;
    let g = *goal;

    let start_item: DijkstraItem<N, C> = DijkstraItem::<N, C> {
        node: s,
        parent: None,
        cost: Zero::zero(),
    };

    let mut goal_item: DijkstraItem<N, C> = DijkstraItem::<N, C> {
        node: g,
        parent: None,
        cost: Zero::zero(),
    };

    let mut open_set: HashMap<N, DijkstraItem<N, C>> = HashMap::new();
    let mut closed_set: HashMap<N, DijkstraItem<N, C>> = HashMap::new();

    open_set.insert(s, start_item);

    while !open_set.is_empty() {
        // TODO: figure out how to not do unwrap or at least throw out errors when no
        // Figure out if dereferncing is copying/cloning
        let cur_node = *open_set
            .iter()
            .min_by(|first, second| first.1.cost.cmp(&second.1.cost))
            .unwrap()
            .0;

        let cur_item = *open_set.get(&cur_node).unwrap();

        if cur_item.node == g {
            is_path_found = true;
            goal_item.parent = cur_item.parent;
            goal_item.cost = cur_item.cost;
            closed_set.insert(goal_item.node, goal_item);
            break;
        }

        open_set.remove(&cur_node);
        closed_set.insert(cur_node, cur_item);

        let neighbors = neighbor_fn(&cur_item.node);

        for (neighbor, neighbor_cost) in neighbors {
            if closed_set.contains_key(&neighbor) {
                continue;
            }

            let neighbor_item: DijkstraItem<N, C> = DijkstraItem::<N, C> {
                node: neighbor,
                parent: Some(cur_node),
                cost: cur_item.cost + neighbor_cost,
            };

            if !open_set.contains_key(&neighbor) {
                open_set.insert(neighbor, neighbor_item);
            } else if neighbor_item.cost < open_set.get(&neighbor).unwrap().cost {
                open_set.insert(neighbor, neighbor_item);
            }
        }
    }

    if is_path_found {
        path = Some(calculate_final_path(&goal_item, &closed_set));
    }

    path
}

fn calculate_final_path<N, C>(
    goal_item: &DijkstraItem<N, C>,
    closed_set: &HashMap<N, DijkstraItem<N, C>>,
) -> Vec<N>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
{
    let mut path: Vec<N> = vec![goal_item.node];

    let mut parent = goal_item.parent;

    while parent.is_some() {
        let neighbor_item = closed_set.get(&parent.unwrap()).unwrap();
        path.push(neighbor_item.node);
        parent = neighbor_item.parent;
    }

    path.reverse();
    path
}

// use std::cmp::Ordering;
// use std::collections::{BinaryHeap, HashMap};

// // Define the possible directions of movement in the grid (up, down, left, right)
// const DIRECTIONS: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

// // A structure to represent a cell in the grid with coordinates and its tentative distance
// #[derive(Clone, PartialEq, Eq)]
// struct Cell {
//     row: i32,
//     col: i32,
//     distance: f64,
// }

// // Implementing Ord and PartialOrd to allow the cell to be used in a BinaryHeap (min-heap by distance)
// impl Ord for Cell {
//     fn cmp(&self, other: &Self) -> Ordering {
//         self.distance.partial_cmp(&other.distance).unwrap()
//     }
// }

// impl PartialOrd for Cell {
//     fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
//         Some(self.cmp(other))
//     }
// }

// type Grid = Vec<Vec<i32>>; // Grid representation (1 for walkable, 0 for blocked)

// fn dijkstra(grid: &Grid, start: (i32, i32), goal: (i32, i32)) -> Option<Vec<(i32, i32)>> {
//     let rows = grid.len() as i32;
//     let cols = grid[0].len() as i32;

//     let mut distances: HashMap<(i32, i32), f64> = HashMap::new();
//     let mut previous: HashMap<(i32, i32), Option<(i32, i32)>> = HashMap::new();
//     let mut pq = BinaryHeap::new();

//     // Initialize distances and previous cells
//     for row in 0..rows {
//         for col in 0..cols {
//             let cell = (row, col);
//             distances.insert(cell, f64::INFINITY);
//             previous.insert(cell, None);
//         }
//     }

//     // Set the start cell distance to 0 and push it onto the priority queue
//     distances.insert(start, 0.0);
//     pq.push(Cell {
//         row: start.0,
//         col: start.1,
//         distance: 0.0,
//     });

//     while let Some(Cell { row, col, distance }) = pq.pop() {
//         let current = (row, col);

//         // If we reached the goal, reconstruct the path
//         if current == goal {
//             let mut path = Vec::new();
//             let mut current = Some(goal);

//             // Reconstruct the path from goal to start
//             while let Some(cell) = current {
//                 path.push(cell);
//                 current = previous.get(&cell).cloned().unwrap();
//             }
//             path.reverse();
//             return Some(path); // Return the path from start to goal
//         }

//         // Explore neighbors (up, down, left, right)
//         for &(dr, dc) in &DIRECTIONS {
//             let (new_row, new_col) = (row + dr, col + dc);
//             // Check bounds and if the cell is walkable
//             if new_row >= 0 && new_row < rows && new_col >= 0 && new_col < cols {
//                 if grid[new_row as usize][new_col as usize] == 1 {
//                     // Only walkable cells
//                     let new_distance = distance + 1.0; // Assuming uniform cost for each move

//                     if new_distance < *distances.get(&(new_row, new_col)).unwrap() {
//                         distances.insert((new_row, new_col), new_distance);
//                         previous.insert((new_row, new_col), Some((row, col)));
//                         pq.push(Cell {
//                             row: new_row,
//                             col: new_col,
//                             distance: new_distance,
//                         });
//                     }
//                 }
//             }
//         }
//     }

//     None // No path found
// }
