use num_traits::Zero;
use std::collections::HashMap;
use std::hash::Hash;

// Define a structure for the priority queue element
/// Item to hold the current node, its parent, and cost to get to that node from starting node
///
/// # Generic Arguments
///
/// * `N` - Node Type (i.e. [i32, i32])
/// * `C` - Cost Type (i.e. u32)
///
/// # Variables
///
/// * `node` - current node
/// * `parent` - parent node, may not exist if starting node
/// * `cost` - cost to move to this node from start
#[derive(Debug, Copy, Clone)]
struct DijkstraItem<N, C> {
    node: N,
    parent: Option<N>,
    cost: C,
}

impl<N, C> Eq for DijkstraItem<N, C> where C: Eq {}

impl<N, C> PartialEq for DijkstraItem<N, C>
where
    C: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl<N, C> Ord for DijkstraItem<N, C>
where
    C: Ord,
{
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost.cmp(&other.cost)
    }
}

impl<N, C> PartialOrd for DijkstraItem<N, C>
where
    C: PartialOrd,
{
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}

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
///
/// # Returns
///
/// A path if one is reachable
pub fn plan<N, C, FF, FN, IT>(start: &N, finish_fn: &mut FF, neighbor_fn: &mut FN) -> Option<Vec<N>>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
    FF: FnMut(&N) -> bool,
    FN: FnMut(&N) -> IT,
    IT: IntoIterator<Item = (N, C)>,
{
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

    let mut is_path_found: bool = false;
    while !open_set.is_empty() {
        // Get the node with the lowest cost
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
        return Some(calculate_final_path(&goal_item, &closed_set));
    }

    None
}

/// Calculates the final path from the start node to the goal node
/// using the `closed_set` and the `goal_item`.
///
/// # Arguments
///
/// * `goal_item`: The Dijkstra item of the goal node
/// * `closed_set`: The set of all nodes that have been visited
///
/// # Returns
///
/// The final path from the start node to the goal node
fn calculate_final_path<N, C>(
    goal_item: &DijkstraItem<N, C>,
    closed_set: &HashMap<N, DijkstraItem<N, C>>,
) -> Vec<N>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
{
    let mut path = Vec::with_capacity(closed_set.len());
    let mut current_item = goal_item;

    // Backtrack to the start node
    while let Some(parent) = current_item.parent {
        path.push(current_item.node);

        // Get the next item in the path
        current_item = closed_set.get(&parent).unwrap();
    }

    // Add the start node to the path
    path.push(current_item.node);

    // Reverse the path to get the order from start to goal
    path.reverse();
    path
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::path_planning::grid_item::{self, Index2D};

    // set 2d map
    fn get_map() -> Vec<Vec<u8>> {
        let mut map = vec![vec![0u8; 5]; 5];

        // make map[4][4] unreachable
        map[3][4] = 1;
        map[4][3] = 1;
        map
    }

    // get possible directions to expand neighbors
    fn get_directions() -> Vec<Index2D<i32>> {
        let dirs = vec![Index2D(1, 0), Index2D(0, 1), Index2D(-1, 0), Index2D(0, -1)];
        dirs
    }

    #[test]
    fn test_dijkstras_unreachable() {
        let map = get_map();
        let dirs = get_directions();
        let start = Index2D(0, 0);
        let end = Index2D(4, 4);
        let bounds = [Index2D(0, 0), Index2D(4, 4)];

        let path = plan(
            &start,
            &mut |node| grid_item::reach_goal(&end, node),
            &mut |node| node.populate_neighbors(&map, &bounds, &dirs),
        );

        assert_eq!(path, None);
    }

    #[test]
    fn test_dijkstras_reachable() {
        let map = get_map();
        let dirs = get_directions();
        let start = Index2D(0, 0);
        let end = Index2D(0, 4);
        let bounds = [Index2D(0, 0), Index2D(4, 4)];

        let path = plan(
            &start,
            &mut |node| grid_item::reach_goal(&end, node),
            &mut |node| node.populate_neighbors(&map, &bounds, &dirs),
        );

        assert_eq!(
            path,
            Some(vec![
                Index2D(0, 0),
                Index2D(0, 1),
                Index2D(0, 2),
                Index2D(0, 3),
                Index2D(0, 4),
            ])
        );
    }
}
