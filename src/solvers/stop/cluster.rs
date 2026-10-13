use crate::common::instance::Instance;

#[derive(Debug, Clone, PartialEq)]
pub struct StopCluster {
    pub id: usize,
    pub original_cluster_id: usize,
    pub node_ids: Vec<usize>,
    pub max_profit: f64,
}

#[derive(Debug, Clone)]
pub struct StopData {
    pub depot_id: usize,
    pub clusters: Vec<StopCluster>,
    pub node_to_cluster: Vec<Option<usize>>,
    pub node_profits: Vec<f64>,
    pub num_customers: usize,
}

impl StopData {
    pub fn new(instance: &Instance) -> Self {
        let depot_id = instance.vehicles.first().map(|v| v.start_node_id).unwrap_or(0);
        let num_nodes = instance.nodes.len();

        // Node profits: from subgroups (in STOP, subgroup i has node i and profit p_i)
        let mut node_profits = vec![0.0; num_nodes];
        for sg in &instance.subgroups {
            for &nid in &sg.node_ids {
                if nid < num_nodes {
                    node_profits[nid] = sg.profit;
                }
            }
        }

        let mut clusters = Vec::new();
        let mut node_to_cluster = vec![None; num_nodes];

        for cluster in &instance.clusters {
            let mut customer_nodes = Vec::new();

            for &sg_id in &cluster.subgroup_ids {
                if let Some(sg) = instance.subgroups.get(sg_id) {
                    for &nid in &sg.node_ids {
                        if nid != depot_id && nid < num_nodes {
                            customer_nodes.push(nid);
                        }
                    }
                }
            }

            if !customer_nodes.is_empty() {
                customer_nodes.sort_unstable();
                customer_nodes.dedup();

                let max_profit = customer_nodes
                    .iter()
                    .map(|&nid| node_profits[nid])
                    .fold(0.0_f64, f64::max);

                let id = clusters.len();
                for &nid in &customer_nodes {
                    node_to_cluster[nid] = Some(id);
                }

                clusters.push(StopCluster {
                    id,
                    original_cluster_id: cluster.id,
                    node_ids: customer_nodes,
                    max_profit,
                });
            }
        }

        let num_customers = if num_nodes > 0 { num_nodes - 1 } else { 0 };

        Self {
            depot_id,
            clusters,
            node_to_cluster,
            node_profits,
            num_customers,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_extract_stop_clusters() {
        let instance = Instance {
            name: "test_stop".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 1.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 2.0, y: 2.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 15.0, node_ids: vec![1], ..Default::default() },
                Subgroup { id: 2, profit: 25.0, node_ids: vec![2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1, 2] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };

        let data = StopData::new(&instance);
        assert_eq!(data.depot_id, 0);
        assert_eq!(data.clusters.len(), 1);
        assert_eq!(data.clusters[0].node_ids, vec![1, 2]);
        assert_eq!(data.clusters[0].max_profit, 25.0);
        assert_eq!(data.node_to_cluster[1], Some(0));
        assert_eq!(data.node_to_cluster[2], Some(0));
        assert_eq!(data.node_to_cluster[0], None);
        assert_eq!(data.node_profits[1], 15.0);
        assert_eq!(data.node_profits[2], 25.0);
    }
}
