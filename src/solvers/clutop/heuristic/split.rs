use std::collections::HashSet;
use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::heuristic::local_search::{calculate_path_cost, two_opt};
use crate::solvers::clutop::heuristic::operators::HeuristicState;

#[derive(Clone)]
struct BeamNode {
    selected_clusters: Vec<usize>,
    score: f64,
    upper_bound: f64,
}

pub fn split_procedure(
    giant_tour: &[usize],
    instance: &Instance,
    data: &CluTopData,
    mut best_state: HeuristicState,
) -> HeuristicState {
    let num_clusters = data.clusters.len();
    if num_clusters == 0 {
        return best_state;
    }

    let mut lb = best_state.total_score;
    let total_possible_profit: f64 = data.clusters.iter().map(|c| c.profit).sum();

    // Sort cluster indices by profit / node_count descending for efficient beam search
    let mut cluster_order: Vec<usize> = (0..num_clusters).collect();
    cluster_order.sort_by(|&a, &b| {
        let eff_a = data.clusters[a].profit / (data.clusters[a].node_ids.len() as f64).max(1.0);
        let eff_b = data.clusters[b].profit / (data.clusters[b].node_ids.len() as f64).max(1.0);
        eff_b.partial_cmp(&eff_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    let beam_width = num_clusters.max(15);

    let root = BeamNode {
        selected_clusters: Vec::new(),
        score: 0.0,
        upper_bound: total_possible_profit,
    };

    let mut curr_list = vec![root];

    for &c_idx in &cluster_order {
        let mut tmp_list: Vec<BeamNode> = Vec::new();
        let cluster_profit = data.clusters[c_idx].profit;

        for node in &curr_list {
            // Branch 1: Include cluster c_idx
            let mut incl_clusters = node.selected_clusters.clone();
            incl_clusters.push(c_idx);
            let incl_score = node.score + cluster_profit;
            let incl_ub = node.upper_bound; // upper bound remains the same

            if incl_ub > lb {
                // Feasibility check (FC)
                if let Some((routes, costs)) = feasibility_check(giant_tour, &incl_clusters, instance, data) {
                    if incl_score > lb {
                        lb = incl_score;
                        best_state.routes = routes;
                        best_state.route_costs = costs;
                        best_state.total_cost = best_state.route_costs.iter().sum();
                        best_state.visited_clusters = incl_clusters.iter().copied().collect();
                        best_state.total_score = incl_score;
                    }

                    tmp_list.push(BeamNode {
                        selected_clusters: incl_clusters,
                        score: incl_score,
                        upper_bound: incl_ub,
                    });
                }
            }

            // Branch 2: Exclude cluster c_idx
            let excl_clusters = node.selected_clusters.clone();
            let excl_score = node.score;
            let excl_ub = node.upper_bound - cluster_profit;

            if excl_ub > lb {
                tmp_list.push(BeamNode {
                    selected_clusters: excl_clusters,
                    score: excl_score,
                    upper_bound: excl_ub,
                });
            }
        }

        if tmp_list.is_empty() {
            break;
        }

        // Keep top beam_width nodes sorted by upper bound and score
        tmp_list.sort_by(|a, b| {
            b.upper_bound
                .partial_cmp(&a.upper_bound)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal))
        });
        tmp_list.truncate(beam_width);

        curr_list = tmp_list;
    }

    best_state
}

/// Feasibility check: extracts customers of selected clusters in giant tour order
/// and packs them greedily into m vehicles.
fn feasibility_check(
    giant_tour: &[usize],
    selected_clusters: &[usize],
    instance: &Instance,
    data: &CluTopData,
) -> Option<(Vec<Vec<usize>>, Vec<f64>)> {
    let mut required_nodes: HashSet<usize> = HashSet::new();
    for &c_idx in selected_clusters {
        for &nid in &data.clusters[c_idx].node_ids {
            required_nodes.insert(nid);
        }
    }

    if required_nodes.is_empty() {
        let num_v = instance.vehicles.len();
        return Some((vec![vec![data.depot_id, data.depot_id]; num_v], vec![0.0; num_v]));
    }

    // Sequence of nodes respecting giant tour order
    let seq: Vec<usize> = giant_tour
        .iter()
        .copied()
        .filter(|nid| required_nodes.contains(nid))
        .collect();

    // Verify all required nodes are present in seq
    if seq.len() < required_nodes.len() {
        return None;
    }

    let num_vehicles = instance.vehicles.len();
    let depot = data.depot_id;
    let mut routes: Vec<Vec<usize>> = Vec::with_capacity(num_vehicles);
    let mut route_costs: Vec<f64> = Vec::with_capacity(num_vehicles);

    let mut current_route = vec![depot];
    let mut current_cost = 0.0;
    let mut vehicle_idx = 0;

    for &cust in &seq {
        if vehicle_idx >= num_vehicles {
            return None; // Exceeded available fleet
        }

        let last_node = *current_route.last().unwrap();
        // Cost if adding cust and returning to depot
        let added_dist = instance.get_distance(last_node, cust);
        let return_dist = instance.get_distance(cust, depot);

        // Previous return distance to subtract
        let prev_return = if current_route.len() > 1 {
            instance.get_distance(last_node, depot)
        } else {
            0.0
        };

        let candidate_cost = current_cost - prev_return + added_dist + return_dist;

        if candidate_cost <= instance.vehicles[vehicle_idx].budget + 1e-6 {
            current_route.push(cust);
            current_cost = candidate_cost;
        } else {
            // Close current route
            if current_route.len() > 1 {
                current_route.push(depot);
                two_opt(instance, &mut current_route);
                let opt_cost = calculate_path_cost(instance, &current_route);
                routes.push(current_route);
                route_costs.push(opt_cost);
            } else {
                // Customer cannot even fit in an empty route!
                return None;
            }

            vehicle_idx += 1;
            if vehicle_idx >= num_vehicles {
                return None;
            }

            // Start new route with this customer
            let dist_to = instance.get_distance(depot, cust);
            let dist_back = instance.get_distance(cust, depot);
            let single_cost = dist_to + dist_back;

            if single_cost <= instance.vehicles[vehicle_idx].budget + 1e-6 {
                current_route = vec![depot, cust];
                current_cost = single_cost;
            } else {
                return None;
            }
        }
    }

    // Close the last route
    if current_route.len() > 1 {
        current_route.push(depot);
        two_opt(instance, &mut current_route);
        let opt_cost = calculate_path_cost(instance, &current_route);
        routes.push(current_route);
        route_costs.push(opt_cost);
    }

    // Pad with empty routes if fleet is not fully used
    while routes.len() < num_vehicles {
        routes.push(vec![depot, depot]);
        route_costs.push(0.0);
    }

    Some((routes, route_costs))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_split_procedure() {
        let instance = Instance {
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
        };

        let data = CluTopData::new(&instance);
        let init_state = HeuristicState::new(&instance, data.depot_id);
        let gt = vec![1, 2];

        let final_state = split_procedure(&gt, &instance, &data, init_state);
        assert_eq!(final_state.total_score, 30.0);
    }
}
