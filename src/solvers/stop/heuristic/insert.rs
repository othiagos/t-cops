use rand::seq::SliceRandom;
use rand::Rng;

use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::heuristic::cost::evaluate_cluster_insertion;
use crate::solvers::stop::heuristic::destroy::StopSolution;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertionOperatorType {
    RandomOrder,
    ProfitBased,
    ProximityBased,
    Cheapest,
    DynamicProfitOverCost,
    Regret2,
}

pub fn repair<R: Rng>(
    sol: &mut StopSolution,
    op: InsertionOperatorType,
    instance: &Instance,
    data: &StopData,
    rng: &mut R,
) {
    match op {
        InsertionOperatorType::RandomOrder
        | InsertionOperatorType::ProfitBased
        | InsertionOperatorType::ProximityBased => {
            predetermined_order_insertion(sol, op, instance, data, rng);
        }
        InsertionOperatorType::Cheapest
        | InsertionOperatorType::DynamicProfitOverCost
        | InsertionOperatorType::Regret2 => {
            parallel_insertion(sol, op, instance, data);
        }
    }
}

fn predetermined_order_insertion<R: Rng>(
    sol: &mut StopSolution,
    op: InsertionOperatorType,
    instance: &Instance,
    data: &StopData,
    rng: &mut R,
) {
    let mut unserved: Vec<usize> = (0..data.clusters.len())
        .filter(|c| !sol.visited_clusters.contains(c))
        .collect();

    match op {
        InsertionOperatorType::RandomOrder => {
            unserved.shuffle(rng);
        }
        InsertionOperatorType::ProfitBased => {
            unserved.sort_by(|&a, &b| {
                data.clusters[b]
                    .max_profit
                    .partial_cmp(&data.clusters[a].max_profit)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        InsertionOperatorType::ProximityBased => {
            unserved.sort_by(|&a, &b| {
                let prox_a = cluster_internal_proximity(a, instance, data);
                let prox_b = cluster_internal_proximity(b, instance, data);
                prox_a.partial_cmp(&prox_b).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        _ => {}
    }

    for cluster in unserved {
        if let Some((best_r, best_pos, new_cost)) = find_best_position(sol, cluster, instance, data) {
            sol.routes[best_r].insert(best_pos, cluster);
            sol.route_costs[best_r] = new_cost;
            sol.visited_clusters.insert(cluster);
            sol.total_score += data.clusters[cluster].max_profit;
            sol.total_cost = sol.route_costs.iter().sum();
        }
    }
}

fn parallel_insertion(
    sol: &mut StopSolution,
    op: InsertionOperatorType,
    instance: &Instance,
    data: &StopData,
) {
    loop {
        let unserved: Vec<usize> = (0..data.clusters.len())
            .filter(|c| !sol.visited_clusters.contains(c))
            .collect();

        if unserved.is_empty() {
            break;
        }

        // For each unserved cluster, find its best insertion across all routes
        let mut candidates: Vec<(usize, usize, usize, f64, f64)> = Vec::new();

        for &cluster in &unserved {
            if let Some((r, pos, new_cost)) = find_best_position(sol, cluster, instance, data) {
                let delta = new_cost - sol.route_costs[r];
                candidates.push((cluster, r, pos, delta, new_cost));
            }
        }

        if candidates.is_empty() {
            break;
        }

        let best_candidate = match op {
            InsertionOperatorType::Cheapest => {
                candidates
                    .into_iter()
                    .min_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))
            }
            InsertionOperatorType::DynamicProfitOverCost => {
                candidates.into_iter().max_by(|a, b| {
                    let ratio_a = data.clusters[a.0].max_profit / a.3.max(0.01);
                    let ratio_b = data.clusters[b.0].max_profit / b.3.max(0.01);
                    ratio_a.partial_cmp(&ratio_b).unwrap_or(std::cmp::Ordering::Equal)
                })
            }
            InsertionOperatorType::Regret2 => {
                let mut best_c = candidates[0];
                let mut max_regret = -1.0;

                for c_cand in &candidates {
                    let cluster = c_cand.0;
                    let (delta1, delta2) = find_two_best_route_deltas(sol, cluster, instance, data);
                    let regret = delta2 - delta1;
                    if regret > max_regret {
                        max_regret = regret;
                        best_c = *c_cand;
                    }
                }
                Some(best_c)
            }
            _ => candidates.into_iter().next(),
        };

        if let Some((cluster, r, pos, _, new_cost)) = best_candidate {
            sol.routes[r].insert(pos, cluster);
            sol.route_costs[r] = new_cost;
            sol.visited_clusters.insert(cluster);
            sol.total_score += data.clusters[cluster].max_profit;
            sol.total_cost = sol.route_costs.iter().sum();
        } else {
            break;
        }
    }
}

fn find_best_position(
    sol: &StopSolution,
    cluster: usize,
    instance: &Instance,
    data: &StopData,
) -> Option<(usize, usize, f64)> {
    let mut best: Option<(usize, usize, f64)> = None;

    for (r, route) in sol.routes.iter().enumerate() {
        let budget = instance.vehicles[r].budget;

        for pos in 0..=route.len() {
            if let Some(new_cost) = evaluate_cluster_insertion(route, pos, cluster, instance, data, budget) {
                let delta = new_cost - sol.route_costs[r];
                if best.as_ref().is_none_or(|b| delta < b.2) {
                    best = Some((r, pos, delta));
                }
            }
        }
    }

    if let Some((r, pos, _)) = best {
        let budget = instance.vehicles[r].budget;
        let new_cost = evaluate_cluster_insertion(&sol.routes[r], pos, cluster, instance, data, budget).unwrap();
        Some((r, pos, new_cost))
    } else {
        None
    }
}

fn find_two_best_route_deltas(
    sol: &StopSolution,
    cluster: usize,
    instance: &Instance,
    data: &StopData,
) -> (f64, f64) {
    let mut deltas: Vec<f64> = Vec::new();

    for (r, route) in sol.routes.iter().enumerate() {
        let budget = instance.vehicles[r].budget;
        let mut best_r_delta = f64::INFINITY;

        for pos in 0..=route.len() {
            if let Some(new_cost) = evaluate_cluster_insertion(route, pos, cluster, instance, data, budget) {
                let delta = new_cost - sol.route_costs[r];
                if delta < best_r_delta {
                    best_r_delta = delta;
                }
            }
        }

        if best_r_delta.is_finite() {
            deltas.push(best_r_delta);
        }
    }

    deltas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    if deltas.is_empty() {
        (f64::INFINITY, f64::INFINITY)
    } else if deltas.len() == 1 {
        (deltas[0], deltas[0])
    } else {
        (deltas[0], deltas[1])
    }
}

fn cluster_internal_proximity(
    c: usize,
    instance: &Instance,
    data: &StopData,
) -> f64 {
    let nodes = &data.clusters[c].node_ids;
    if nodes.len() <= 1 {
        return 0.0;
    }

    let mut sum = 0.0;
    for &u in nodes {
        for &v in nodes {
            sum += instance.get_distance(u, v);
        }
    }

    sum / (nodes.len() * nodes.len()) as f64
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
    fn test_repair_profit_based() {
        let instance = create_test_instance();
        let data = StopData::new(&instance);
        let mut sol = StopSolution::new(&instance);

        let mut rng = thread_rng();
        repair(&mut sol, InsertionOperatorType::ProfitBased, &instance, &data, &mut rng);

        assert_eq!(sol.total_score, 30.0);
        assert_eq!(sol.visited_clusters.len(), 2);
    }
}
