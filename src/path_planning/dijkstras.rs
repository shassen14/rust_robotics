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

// TODO: None of this will work with floating types from the rip
/// Plans a path from start to goal using the children to expand the nodes
///
/// # Generic Arguments
///
/// * `N` - Node Type (i.e. [i32, i32])
/// * `C` - Cost Type (i.e. u32)
/// * `FF` - Function to signal when we reached the goal or "finished"
/// * `FN` - Function to generate neighbor nodes
/// * `IT` - Iterator Item for FN to generate as a pair (N, C)
///
/// # Arguments
///
/// * `start` - Starting node for the path
/// * `finish_fn` - Function to signal we are finished and can generate a path
/// * `neighbor_fn` - Function to generate neighbors given the current node
/// * Returns a path if one is reachable
///
pub fn plan<N, C, FF, FN, IT>(start: &N, finish_fn: &mut FF, neighbor_fn: &mut FN) -> Option<Vec<N>>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
    FF: FnMut(&N) -> bool,
    FN: FnMut(&N) -> IT,
    IT: IntoIterator<Item = (N, C)>,
{
    let mut path: Option<Vec<N>> = Option::default();
    let mut is_path_found: bool = false;
    let s = *start;

    let start_item: DijkstraItem<N, C> = DijkstraItem::<N, C> {
        node: s,
        parent: None,
        cost: Zero::zero(),
    };

    let mut goal_item: DijkstraItem<N, C> = DijkstraItem::<N, C> {
        node: s,
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

        if finish_fn(&cur_node) {
            is_path_found = true;
            goal_item.node = cur_node;
            goal_item.parent = cur_item.parent;
            goal_item.cost = cur_item.cost;
            closed_set.insert(cur_node, cur_item);
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

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
}
