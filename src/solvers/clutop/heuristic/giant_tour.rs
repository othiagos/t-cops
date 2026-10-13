use std::collections::HashSet;
use rand::Rng;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::heuristic::operators::HeuristicState;

pub fn construct_giant_tour<R: Rng>(
    state: &HeuristicState,
    data: &CluTopData,
    num_total_nodes: usize,
    rng: &mut R,
) -> Vec<usize> {
    let mut gt = Vec::new();
    let mut seen = HashSet::new();
    seen.insert(data.depot_id);

    // Concatenate the customer visits from the routes
    for route in &state.routes {
        for &node in route {
            if !seen.contains(&node) {
                seen.insert(node);
                gt.push(node);
            }
        }
    }

    // Insert all unrouted customers in random positions
    for node in 0..num_total_nodes {
        if !seen.contains(&node) {
            seen.insert(node);
            let pos = if gt.is_empty() {
                0
            } else {
                rng.gen_range(0..=gt.len())
            };
            gt.insert(pos, node);
        }
    }

    gt
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::thread_rng;
    use crate::common::instance::{Cluster, Instance, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_construct_giant_tour() {
        let instance = Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 1.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 2.0, y: 2.0, z: 0.0 }, ..Default::default() },
                Node { id: 3, point: Point3 { x: 3.0, y: 3.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1, 2], ..Default::default() },
                Subgroup { id: 2, profit: 20.0, node_ids: vec![3], ..Default::default() },
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
        let mut state = HeuristicState::new(&instance, data.depot_id);
        state.routes[0] = vec![0, 1, 2, 0];

        let mut rng = thread_rng();
        let gt = construct_giant_tour(&state, &data, instance.nodes.len(), &mut rng);

        assert_eq!(gt.len(), 3); // 1, 2, 3
        assert!(gt.contains(&1));
        assert!(gt.contains(&2));
        assert!(gt.contains(&3));
        assert!(!gt.contains(&0));
    }
}
