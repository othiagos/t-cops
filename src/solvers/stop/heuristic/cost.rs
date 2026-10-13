use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;

/// Computes the optimal travel cost and optimal node sequence for a cluster route.
pub fn compute_route_cost_and_path(
    cluster_ids: &[usize],
    instance: &Instance,
    data: &StopData,
) -> (f64, Vec<usize>) {
    let depot = data.depot_id;
    if cluster_ids.is_empty() {
        return (0.0, vec![depot, depot]);
    }

    let k = cluster_ids.len();

    let mut g: Vec<Vec<f64>> = Vec::with_capacity(k);
    let mut best_prev: Vec<Vec<usize>> = Vec::with_capacity(k);

    // Step 0: first cluster from depot
    let first_nodes = &data.clusters[cluster_ids[0]].node_ids;
    let mut g_0 = Vec::with_capacity(first_nodes.len());
    for &u in first_nodes {
        g_0.push(instance.get_distance(depot, u));
    }
    g.push(g_0);
    best_prev.push(vec![0; first_nodes.len()]);

    // Steps 1..k-1: transition between consecutive clusters
    for step in 1..k {
        let prev_nodes = &data.clusters[cluster_ids[step - 1]].node_ids;
        let curr_nodes = &data.clusters[cluster_ids[step]].node_ids;

        let mut g_curr = Vec::with_capacity(curr_nodes.len());
        let mut prev_curr = Vec::with_capacity(curr_nodes.len());

        for &v in curr_nodes {
            let mut min_val = f64::INFINITY;
            let mut min_idx = 0;

            for (p_idx, &u) in prev_nodes.iter().enumerate() {
                let cost = g[step - 1][p_idx] + instance.get_distance(u, v);
                if cost < min_val {
                    min_val = cost;
                    min_idx = p_idx;
                }
            }

            g_curr.push(min_val);
            prev_curr.push(min_idx);
        }

        g.push(g_curr);
        best_prev.push(prev_curr);
    }

    // Return to depot from the last cluster
    let last_nodes = &data.clusters[cluster_ids[k - 1]].node_ids;
    let mut total_cost = f64::INFINITY;
    let mut best_last_idx = 0;

    for (l_idx, &u) in last_nodes.iter().enumerate() {
        let cost = g[k - 1][l_idx] + instance.get_distance(u, depot);
        if cost < total_cost {
            total_cost = cost;
            best_last_idx = l_idx;
        }
    }

    // Traceback the optimal customer sequence
    let mut chosen_nodes = vec![0; k];
    let mut curr_idx = best_last_idx;

    for step in (0..k).rev() {
        chosen_nodes[step] = data.clusters[cluster_ids[step]].node_ids[curr_idx];
        if step > 0 {
            curr_idx = best_prev[step][curr_idx];
        }
    }

    let mut path = Vec::with_capacity(k + 2);
    path.push(depot);
    path.extend(chosen_nodes);
    path.push(depot);

    (total_cost, path)
}

/// Computes only the cost of a route of clusters (faster when path is not needed)
pub fn compute_route_cost(
    cluster_ids: &[usize],
    instance: &Instance,
    data: &StopData,
) -> f64 {
    compute_route_cost_and_path(cluster_ids, instance, data).0
}

/// Evaluates the insertion of candidate_cluster at position `pos` in a route
pub fn evaluate_cluster_insertion(
    cluster_ids: &[usize],
    pos: usize,
    candidate_cluster: usize,
    instance: &Instance,
    data: &StopData,
    max_budget: f64,
) -> Option<f64> {
    let mut new_clusters = Vec::with_capacity(cluster_ids.len() + 1);
    new_clusters.extend_from_slice(&cluster_ids[..pos]);
    new_clusters.push(candidate_cluster);
    new_clusters.extend_from_slice(&cluster_ids[pos..]);

    let new_cost = compute_route_cost(&new_clusters, instance, data);
    if new_cost <= max_budget + 1e-6 {
        Some(new_cost)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 4.0, y: 0.0, z: 0.0 }, ..Default::default() },
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
    fn test_dp_route_cost() {
        let instance = create_test_instance();
        let data = StopData::new(&instance);

        let (cost, path) = compute_route_cost_and_path(&[0, 1], &instance, &data);
        assert_eq!(path, vec![0, 1, 2, 0]);
        // 0 -> (0,3) is 3, (0,3) -> (4,0) is 5, (4,0) -> (0,0) is 4. Total = 12.
        assert!((cost - 12.0).abs() < 1e-4);
    }
}
