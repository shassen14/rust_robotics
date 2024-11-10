use num_traits::Zero;
use rustc_hash::FxHashMap;
use std::hash::Hash;

/***
 * N: Node type
 * C: Cost type
 */
#[derive(Clone, Copy)]
struct DijkstraItem<N, C> {
    pub node: N,
    pub parent: Option<N>,
    pub cost: C,
}

fn calculate_final_path<N, C>(
    goal_item: &DijkstraItem<N, C>,
    closed_set: &FxHashMap<N, DijkstraItem<N, C>>,
) -> Vec<N>
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
{
    let mut path: Vec<N> = vec![goal_item.node];

    let mut parent = goal_item.parent;

    while parent.is_some() {
        let node_item = closed_set.get(&parent.unwrap()).unwrap();
        path.push(node_item.node);
        parent = node_item.parent;
    }

    path.into_iter().rev().collect()
}

// TODO: Lot of copying. Reduce the amount
pub fn plan<N, C, FN, IN>(start: &N, goal: &N, children_fn: &mut FN) -> Option<Vec<N>>
where
    N: Hash + Eq + Copy + std::fmt::Debug,
    C: Zero + Ord + Copy + std::fmt::Debug,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
{
    // Default is none and false
    let mut path: Option<Vec<N>> = Option::default();

    let start_item = DijkstraItem::<N, C> {
        node: start.clone(),
        parent: None,
        cost: Zero::zero(),
    };

    let mut goal_item = DijkstraItem::<N, C> {
        node: goal.clone(),
        parent: None,
        cost: Zero::zero(),
    };

    let mut open_set: FxHashMap<N, DijkstraItem<N, C>> = FxHashMap::default();
    let mut closed_set: FxHashMap<N, DijkstraItem<N, C>> = FxHashMap::default();

    open_set.insert(start.clone(), start_item);

    while !open_set.is_empty() {
        if plan_once(&mut open_set, &mut closed_set, children_fn, &mut goal_item).1 {
            break;
        }
    }

    if closed_set.contains_key(goal) {
        path = Some(calculate_final_path(&goal_item, &closed_set));
    }

    path
}

// TODO: Lot of copying. Reduce the amount
/// This is intended for graphing purposes. It's very inefficient, but good to show what nodes are being
/// populated at each time step.
fn plan_once<N, C, FN, IN>(
    open_set: &mut FxHashMap<N, DijkstraItem<N, C>>,
    closed_set: &mut FxHashMap<N, DijkstraItem<N, C>>,
    children_fn: &mut FN,
    goal_item: &mut DijkstraItem<N, C>,
) -> (Option<Vec<N>>, bool)
where
    N: Hash + Eq + Copy,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
{
    // should be safe to unwrap because of the while loop conditional statement

    let min_cost_index = *open_set
        .iter()
        .min_by(|a, b| a.1.cost.cmp(&b.1.cost))
        .unwrap()
        .0;

    let current_item = *open_set.get(&min_cost_index).unwrap();

    if current_item.node == goal_item.node {
        println!("Found goal");
        goal_item.parent = current_item.parent;
        goal_item.cost = current_item.cost;
        closed_set.insert(goal_item.node, *goal_item);
        return (
            Some(closed_set.clone().keys().into_iter().map(|i| *i).collect()),
            true,
        );
    }

    open_set.remove(&min_cost_index);
    closed_set.insert(min_cost_index.clone(), current_item.clone());

    // Determine the valid children using the function given
    let children = children_fn(&current_item.node);

    for (child, child_cost) in children {
        // if the child is already in the set, no need to check because the cost
        // would be more than what is already in the list
        if closed_set.contains_key(&child) {
            continue;
        }

        let child_item = DijkstraItem::<N, C> {
            node: child,
            parent: Some(current_item.node),
            cost: current_item.cost + child_cost,
        };

        // if child has never been seen before, add to the open list
        // if seen and the child has less cost than previously, change the node to reflect
        // a cheaper path
        if !open_set.contains_key(&child) {
            open_set.insert(child, child_item);
        } else {
            if child_item.cost < open_set.get(&child).unwrap().cost {
                open_set.insert(child, child_item);
            }
        }
    }

    (
        Some(closed_set.clone().keys().into_iter().map(|i| *i).collect()),
        false,
    )
}