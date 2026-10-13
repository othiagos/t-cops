use std::collections::HashSet;
use rand::seq::SliceRandom;
use rand::Rng;

use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::heuristic::cost::compute_route_cost;

#[derive(Debug, Clone)]
pub struct StopSolution {
    pub routes: Vec<Vec<usize>>, // cluster IDs per vehicle
    pub route_costs: Vec<f64>,
    pub visited_clusters: HashSet<usize>,
    pub total_score: f64,
    pub total_cost: f64,
}

impl StopSolution {
    pub fn new(instance: &Instance) -> Self {
        let num_vehicles = instance.vehicles.len();
        Self {
            routes: vec![Vec::new(); num_vehicles],
            route_costs: vec![0.0; num_vehicles],
            visited_clusters: HashSet::new(),
            total_score: 0.0,
            total_cost: 0.0,
        }
    }

    pub fn recompute(&mut self, instance: &Instance, data: &StopData) {
        self.total_cost = 0.0;
        self.visited_clusters.clear();

        for (r, route) in self.routes.iter().enumerate() {
            let cost = compute_route_cost(route, instance, data);
            self.route_costs[r] = cost;
            self.total_cost += cost;
            for &c in route {
                self.visited_clusters.insert(c);
            }
        }

        self.total_score = self
            .visited_clusters
            .iter()
            .map(|&c| data.clusters[c].max_profit)
            .sum();
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemovalOperatorType {
    Random,
    WorstCost,
    WorstProfit,
    WorstProfitOverCost,
    Related,
}

pub fn destroy<R: Rng>(
    sol: &mut StopSolution,
    op: RemovalOperatorType,
    instance: &Instance,
    data: &StopData,
    rng: &mut R,
) {
    if sol.visited_clusters.is_empty() {
        return;
    }

    let k = data.clusters.len();
    let max_remove = 4.max(((0.3 * k as f64) as usize).min(40));
    let num_to_remove = rng.gen_range(1..=max_remove).min(sol.visited_clusters.len());

    let removed_clusters = match op {
        RemovalOperatorType::Random => {
            let mut visited: Vec<usize> = sol.visited_clusters.iter().copied().collect();
            visited.shuffle(rng);
            visited.truncate(num_to_remove);
            visited
        }
        RemovalOperatorType::WorstProfit => {
            let mut visited: Vec<usize> = sol.visited_clusters.iter().copied().collect();

            visited.sort_by(|&a, &b| {
                data.clusters[a]
                    .max_profit
                    .partial_cmp(&data.clusters[b].max_profit)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            random_power_selection(&visited, num_to_remove, 10.0, rng)
        }
        RemovalOperatorType::WorstCost => {
            let mut cluster_savings = compute_removal_savings(sol, instance, data);

            cluster_savings.sort_by(|a, b| {
                b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal)
            });

            let sorted_clusters: Vec<usize> = cluster_savings.into_iter().map(|(c, _)| c).collect();
            random_power_selection(&sorted_clusters, num_to_remove, 5.0, rng)
        }
        RemovalOperatorType::WorstProfitOverCost => {
            let mut cluster_savings = compute_removal_savings(sol, instance, data);

            cluster_savings.sort_by(|a, b| {
                let ratio_a = data.clusters[a.0].max_profit / a.1.max(0.01);
                let ratio_b = data.clusters[b.0].max_profit / b.1.max(0.01);
                ratio_a.partial_cmp(&ratio_b).unwrap_or(std::cmp::Ordering::Equal)
            });

            let sorted_clusters: Vec<usize> = cluster_savings.into_iter().map(|(c, _)| c).collect();
            
            random_power_selection(&sorted_clusters, num_to_remove, 4.0, rng)
        }
        RemovalOperatorType::Related => {
            remove_related_clusters(sol, num_to_remove, instance, data, rng)
        }
    };

    let removed_set: HashSet<usize> = removed_clusters.into_iter().collect();

    for route in &mut sol.routes {
        route.retain(|c| !removed_set.contains(c));
    }

    sol.recompute(instance, data);
}

fn compute_removal_savings(
    sol: &StopSolution,
    instance: &Instance,
    data: &StopData,
) -> Vec<(usize, f64)> {
    let mut savings = Vec::new();

    for (r, route) in sol.routes.iter().enumerate() {
        let base_cost = sol.route_costs[r];

        for (pos, &cluster) in route.iter().enumerate() {
            let mut without_c = route.clone();
            without_c.remove(pos);
            let cost_without = compute_route_cost(&without_c, instance, data);
            let saved = base_cost - cost_without;
            savings.push((cluster, saved));
        }
    }

    savings
}

fn random_power_selection<R: Rng>(
    sorted_list: &[usize],
    count: usize,
    rp: f64,
    rng: &mut R,
) -> Vec<usize> {
    let mut pool = sorted_list.to_vec();
    let mut selected = Vec::with_capacity(count);

    while selected.len() < count && !pool.is_empty() {
        let y: f64 = rng.gen_range(0.0..1.0);
        let idx = (y.powf(rp) * pool.len() as f64) as usize;
        let idx = idx.min(pool.len() - 1);
        selected.push(pool.remove(idx));
    }

    selected
}

fn remove_related_clusters<R: Rng>(
    sol: &StopSolution,
    count: usize,
    instance: &Instance,
    data: &StopData,
    rng: &mut R,
) -> Vec<usize> {
    let mut visited: Vec<usize> = sol.visited_clusters.iter().copied().collect();
    if visited.is_empty() {
        return Vec::new();
    }

    let mut selected = Vec::with_capacity(count);
    let start_idx = rng.gen_range(0..visited.len());
    selected.push(visited.remove(start_idx));

    while selected.len() < count && !visited.is_empty() {
        let last_sel = *selected.last().unwrap();
        let mut min_rel = f64::INFINITY;
        let mut best_v_idx = 0;

        for (idx, &v) in visited.iter().enumerate() {
            let dist = average_cluster_distance(last_sel, v, instance, data);
            if dist < min_rel {
                min_rel = dist;
                best_v_idx = idx;
            }
        }

        selected.push(visited.remove(best_v_idx));
    }

    selected
}

fn average_cluster_distance(
    c1: usize,
    c2: usize,
    instance: &Instance,
    data: &StopData,
) -> f64 {
    let nodes1 = &data.clusters[c1].node_ids;
    let nodes2 = &data.clusters[c2].node_ids;
    if nodes1.is_empty() || nodes2.is_empty() {
        return 0.0;
    }

    let mut sum = 0.0;
    for &u in nodes1 {
        for &v in nodes2 {
            sum += instance.get_distance(u, v);
        }
    }

    sum / (nodes1.len() * nodes2.len()) as f64
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
    fn test_destroy_operators() {
        let instance = create_test_instance();
        let data = StopData::new(&instance);
        let mut sol = StopSolution::new(&instance);
        sol.routes[0] = vec![0, 1];
        sol.recompute(&instance, &data);

        let mut rng = thread_rng();
        destroy(&mut sol, RemovalOperatorType::Random, &instance, &data, &mut rng);
        assert!(sol.visited_clusters.len() < 2);
    }
}
