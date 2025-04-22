use std::collections::HashMap;

/// A trait for path planning algorithms
/// A trait to define the interface for path planning algorithms
///
/// Implementors of this trait must provide methods to set the start and goal
/// nodes and execute the planning algorithm.
pub trait PathPlanner<N, Cost>
where
    N: Eq + std::hash::Hash + Clone,
    Cost: PartialOrd + Copy,
{
    /// Set the start and goal nodes for the planning algorithm
    ///
    /// The start node is the node from which the algorithm will begin
    /// searching for a path. The goal node is the node to which the algorithm
    /// should search for a path to.
    fn set_start_goal(&mut self, start: N, goal: N);

    /// Execute the planning algorithm
    ///
    /// This method will execute the planning algorithm and return an optional
    /// vector of nodes which represent the shortest path from the start node to
    /// the goal node. If the algorithm could not find a path, it will return
    /// `None`.
    fn plan(&mut self) -> Option<Vec<N>>;

    /// This method is used to access internal data structures containing
    /// information about the cost of reaching a node or the nodes that have
    /// been visited. The method is not required to be implemented and will
    /// return `None` by default.
    fn get_cost_map(&self) -> Option<&HashMap<N, Cost>> {
        None
    }
}

/// A trait to define the search space for a path planner
///
/// The search space is responsible for providing neighbors of a node
/// and their associated costs.
///
/// Implementors of this trait must provide a method to return the
/// neighbors of a given node.
pub trait SearchSpace<N, Cost>
where
    N: Eq + std::hash::Hash + Clone,
    Cost: PartialOrd + Copy,
{
    /// The neighbors are returned as a vector of tuples, where the first
    /// element of the tuple is the neighbor node and the second element is
    /// the cost to move from the given node to the neighbor node.
    fn neighbors(&self, node: &N) -> Vec<(N, Cost)>;
}
