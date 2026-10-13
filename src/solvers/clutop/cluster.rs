use std::collections::HashSet;
use crate::common::instance::Instance;

#[derive(Debug, Clone, PartialEq)]
pub struct CluTopCluster {
    pub id: usize,
    pub original_cluster_id: usize,
    pub profit: f64,
    pub node_ids: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct CluTopData {
    pub depot_id: usize,
    pub clusters: Vec<CluTopCluster>,
    pub customer_clusters: Vec<Vec<usize>>,
    pub num_customers: usize,
}

impl CluTopData {
    pub fn new(instance: &Instance) -> Self {
        let depot_id = instance.vehicles.first().map(|v| v.start_node_id).unwrap_or(0);
        let mut clusters = Vec::new();

        for cluster in &instance.clusters {
            let mut node_set = HashSet::new();
            let mut profit = 0.0;

            for &sg_id in &cluster.subgroup_ids {
                if let Some(sg) = instance.subgroups.get(sg_id) {
                    profit += sg.profit;
                    for &nid in &sg.node_ids {
                        if nid != depot_id {
                            node_set.insert(nid);
                        }
                    }
                }
            }

            if !node_set.is_empty() && profit > 0.0 {
                let mut node_ids: Vec<usize> = node_set.into_iter().collect();
                node_ids.sort_unstable();

                let id = clusters.len();
                clusters.push(CluTopCluster {
                    id,
                    original_cluster_id: cluster.id,
                    profit,
                    node_ids,
                });
            }
        }

        let num_nodes = instance.nodes.len();
        let mut customer_clusters: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
        for cluster in &clusters {
            for &node_id in &cluster.node_ids {
                if node_id < num_nodes {
                    customer_clusters[node_id].push(cluster.id);
                }
            }
        }

        let num_customers = if num_nodes > 0 { num_nodes - 1 } else { 0 };

        Self {
            depot_id,
            clusters,
            customer_clusters,
            num_customers,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_extract_clutop_clusters() {
        let instance = Instance {
            name: "test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 1.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 2.0, y: 2.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 100.0, node_ids: vec![1, 2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };

        let data = CluTopData::new(&instance);
        assert_eq!(data.depot_id, 0);
        assert_eq!(data.clusters.len(), 1);
        assert_eq!(data.clusters[0].profit, 100.0);
        assert_eq!(data.clusters[0].node_ids, vec![1, 2]);
        assert_eq!(data.customer_clusters[1], vec![0]);
        assert_eq!(data.customer_clusters[2], vec![0]);
        assert_eq!(data.customer_clusters[0], Vec::<usize>::new());
    }
}
