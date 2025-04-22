use crate::path_planning::base::{PathPlanner, SearchSpace};
use std::collections::BinaryHeap;
use std::collections::HashMap;

pub struct DijkstraPlanner<N, C, S>
where
    N: Eq + std::hash::Hash + Clone,
    C: PartialOrd + Copy + Default + std::ops::Add<Output = C>,
    S: SearchSpace<N, C>,
{
    start: Option<N>,
    goal: Option<N>,
    space: S,
    cost_map: HashMap<N, C>,
    came_from: HashMap<N, N>,
}

impl<N, C, S> DijkstraPlanner<N, C, S>
where
    N: Eq + std::hash::Hash + Clone,
    C: PartialOrd + Copy + Default + std::ops::Add<Output = C>,
    S: SearchSpace<N, C>,
{
    pub fn new(space: S) -> Self {
        Self {
            start: None,
            goal: None,
            space,
            cost_map: HashMap::new(),
            came_from: HashMap::new(),
        }
    }
}

impl<N, C, S> PathPlanner<N, C> for DijkstraPlanner<N, C, S>
where
    N: Ord + Eq + std::hash::Hash + Clone,
    C: Ord + PartialOrd + Copy + Default + std::ops::Add<Output = C>,
    S: SearchSpace<N, C>,
{
    fn set_start_goal(&mut self, start: N, goal: N) {
        self.start = Some(start);
        self.goal = Some(goal);
    }

    fn plan(&mut self) -> Option<Vec<N>> {
        let start = self.start.clone()?;
        let goal = self.goal.clone()?;

        let mut frontier = BinaryHeap::new();
        frontier.push(std::cmp::Reverse((C::default(), start.clone())));

        self.cost_map.insert(start.clone(), C::default());
        self.came_from.insert(start.clone(), start.clone());

        while let Some(std::cmp::Reverse((cost_so_far, current))) = frontier.pop() {
            if current == goal {
                // Reconstruct path
                let mut path = vec![goal.clone()];
                let mut node = goal;
                while let Some(prev) = self.came_from.get(&node) {
                    if prev == &node {
                        break;
                    }
                    path.push(prev.clone());
                    node = prev.clone();
                }
                path.reverse();
                return Some(path);
            }

            for (neighbor, cost) in self.space.neighbors(&current) {
                let new_cost = cost_so_far + cost;
                if self.cost_map.get(&neighbor).map_or(true, |c| new_cost < *c) {
                    self.cost_map.insert(neighbor.clone(), new_cost);
                    self.came_from.insert(neighbor.clone(), current.clone());
                    frontier.push(std::cmp::Reverse((new_cost, neighbor)));
                }
            }
        }

        None
    }
}
