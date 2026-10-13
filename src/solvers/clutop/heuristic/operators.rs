use std::collections::HashSet;
use rand::seq::SliceRandom;
use rand::Rng;

use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::heuristic::local_search::two_opt;

#[derive(Debug, Clone)]
pub struct HeuristicState {
    pub routes: Vec<Vec<usize>>,
    pub route_costs: Vec<f64>,
    pub visited_clusters: HashSet<usize>,
    pub total_score: f64,
    pub total_cost: f64,
}

impl HeuristicState {
    pub fn new(instance: &Instance, depot_id: usize) -> Self {
        let num_vehicles = instance.vehicles.len();
        let routes = vec![vec![depot_id, depot_id]; num_vehicles];
        let route_costs = vec![0.0; num_vehicles];
        Self {
            routes,
            route_costs,
            visited_clusters: HashSet::new(),
            total_score: 0.0,
            total_cost: 0.0,
        }
    }

    pub fn recompute_costs(&mut self, instance: &Instance) {
        self.total_cost = 0.0;
        for (r, route) in self.routes.iter_mut().enumerate() {
            let cost = two_opt(instance, route);
            self.route_costs[r] = cost;
            self.total_cost += cost;
        }
    }

    pub fn recompute_score(&mut self, data: &CluTopData) {
        self.total_score = self
            .visited_clusters
            .iter()
            .map(|&c_idx| data.clusters[c_idx].profit)
            .sum();
    }

    pub fn is_feasible(&self, instance: &Instance) -> bool {
        for (r, &cost) in self.route_costs.iter().enumerate() {
            if cost > instance.vehicles[r].budget + 1e-6 {
                return false;
            }
        }
        true
    }

    pub fn remove_unneeded_nodes(&mut self, instance: &Instance, data: &CluTopData) {
        let mut required_nodes: HashSet<usize> = HashSet::new();
        for &c_idx in &self.visited_clusters {
            for &nid in &data.clusters[c_idx].node_ids {
                required_nodes.insert(nid);
            }
        }

        let depot = data.depot_id;
        for route in &mut self.routes {
            route.retain(|&node| node == depot || required_nodes.contains(&node));
            if route.len() == 1 {
                route.push(depot);
            }
        }

        self.recompute_costs(instance);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertionOperatorType {
    Best,
    RandomBest,
    Regret2,
}

pub fn destroy<R: Rng>(
    state: &mut HeuristicState,
    d_max: usize,
    instance: &Instance,
    data: &CluTopData,
    rng: &mut R,
) {
    if state.visited_clusters.is_empty() {
        return;
    }

    let mut visited: Vec<usize> = state.visited_clusters.iter().copied().collect();
    visited.shuffle(rng);

    let num_to_remove = d_max.min(visited.len());
    for c_id in &visited[..num_to_remove] {
        state.visited_clusters.remove(c_id);
    }

    state.remove_unneeded_nodes(instance, data);
    state.recompute_score(data);
}

pub fn repair<R: Rng>(
    state: &mut HeuristicState,
    op: InsertionOperatorType,
    instance: &Instance,
    data: &CluTopData,
    rng: &mut R,
) {
    let mut unrouted: Vec<usize> = (0..data.clusters.len())
        .filter(|c| !state.visited_clusters.contains(c))
        .collect();

    unrouted.shuffle(rng);

    for c_idx in unrouted {
        let cluster = &data.clusters[c_idx];

        // Find which nodes of this cluster are currently not in any route
        let mut routed_nodes = HashSet::new();
        for route in &state.routes {
            for &nid in route {
                routed_nodes.insert(nid);
            }
        }

        let missing_nodes: Vec<usize> = cluster
            .node_ids
            .iter()
            .copied()
            .filter(|nid| !routed_nodes.contains(nid))
            .collect();

        if missing_nodes.is_empty() {
            // All nodes are already present! Just collect the cluster
            state.visited_clusters.insert(c_idx);
            state.total_score += cluster.profit;
            continue;
        }

        // Try to insert all missing nodes
        let mut backup_routes = state.routes.clone();
        let mut backup_costs = state.route_costs.clone();
        let mut remaining_missing = missing_nodes;
        let mut success = true;

        while !remaining_missing.is_empty() {
            let next_node_idx = match op {
                InsertionOperatorType::RandomBest => {
                    rng.gen_range(0..remaining_missing.len())
                }
                InsertionOperatorType::Best => {
                    // Pick the node with the minimum best insertion cost
                    let mut best_cost_delta = f64::INFINITY;
                    let mut best_idx = 0;

                    for (idx, &nid) in remaining_missing.iter().enumerate() {
                        if let Some((_, _, delta)) = find_best_insertion(nid, &backup_routes, instance)
                            && delta < best_cost_delta
                        {
                            best_cost_delta = delta;
                            best_idx = idx;
                        }
                    }
                    best_idx
                }
                InsertionOperatorType::Regret2 => {
                    // Pick the node with maximum regret (cost2 - cost1)
                    let mut max_regret = -1.0;
                    let mut best_idx = 0;

                    for (idx, &nid) in remaining_missing.iter().enumerate() {
                        let (best_delta, second_delta) = find_two_best_insertions(nid, &backup_routes, instance);
                        let regret = second_delta - best_delta;
                        if regret > max_regret {
                            max_regret = regret;
                            best_idx = idx;
                        }
                    }
                    best_idx
                }
            };

            let node_to_insert = remaining_missing.remove(next_node_idx);

            if let Some((route_idx, pos, delta)) = find_best_insertion(node_to_insert, &backup_routes, instance) {
                let new_route_cost = backup_costs[route_idx] + delta;
                if new_route_cost <= instance.vehicles[route_idx].budget + 1e-6 {
                    backup_routes[route_idx].insert(pos, node_to_insert);
                    backup_costs[route_idx] = new_route_cost;
                } else {
                    success = false;
                    break;
                }
            } else {
                success = false;
                break;
            }
        }

        if success {
            // Apply 2-opt to improve routes that were modified
            for (r, route) in backup_routes.iter_mut().enumerate() {
                backup_costs[r] = two_opt(instance, route);
            }

            // Verify budgets again after 2-opt (2-opt only decreases cost)
            let mut feasible = true;
            for (r, &cost) in backup_costs.iter().enumerate() {
                if cost > instance.vehicles[r].budget + 1e-6 {
                    feasible = false;
                    break;
                }
            }

            if feasible {
                state.routes = backup_routes;
                state.route_costs = backup_costs;
                state.total_cost = state.route_costs.iter().sum();
                state.visited_clusters.insert(c_idx);
                state.total_score += cluster.profit;
            }
        }
    }
}

/// Finds the best insertion position for a node across all routes: (route_idx, pos, delta_cost)
fn find_best_insertion(
    node: usize,
    routes: &[Vec<usize>],
    instance: &Instance,
) -> Option<(usize, usize, f64)> {
    let mut best: Option<(usize, usize, f64)> = None;

    for (r, route) in routes.iter().enumerate() {
        for pos in 1..route.len() {
            let u = route[pos - 1];
            let v = route[pos];
            let delta = instance.get_distance(u, node) + instance.get_distance(node, v) - instance.get_distance(u, v);

            if best.as_ref().is_none_or(|b| delta < b.2) {
                best = Some((r, pos, delta));
            }
        }
    }

    best
}

/// Finds best and second best insertion costs across all routes
fn find_two_best_insertions(
    node: usize,
    routes: &[Vec<usize>],
    instance: &Instance,
) -> (f64, f64) {
    let mut best1 = f64::INFINITY;
    let mut best2 = f64::INFINITY;

    for route in routes {
        for pos in 1..route.len() {
            let u = route[pos - 1];
            let v = route[pos];
            let delta = instance.get_distance(u, node) + instance.get_distance(node, v) - instance.get_distance(u, v);

            if delta < best1 {
                best2 = best1;
                best1 = delta;
            } else if delta < best2 {
                best2 = delta;
            }
        }
    }

    if best2.is_infinite() {
        best2 = best1;
    }

    (best1, best2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::thread_rng;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 1.0, y: 0.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1], ..Default::default() },
                Subgroup { id: 2, profit: 20.0, node_ids: vec![2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1] },
                Cluster { id: 2, subgroup_ids: vec![2] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_repair_and_destroy() {
        let instance = create_test_instance();
        let data = CluTopData::new(&instance);
        let mut state = HeuristicState::new(&instance, data.depot_id);
        let mut rng = thread_rng();

        repair(&mut state, InsertionOperatorType::Best, &instance, &data, &mut rng);
        assert_eq!(state.total_score, 30.0);
        assert_eq!(state.visited_clusters.len(), 2);

        destroy(&mut state, 1, &instance, &data, &mut rng);
        assert!(state.visited_clusters.len() <= 1);
    }
}
