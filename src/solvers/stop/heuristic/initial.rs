use rand::thread_rng;
use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::heuristic::destroy::StopSolution;
use crate::solvers::stop::heuristic::insert::{repair, InsertionOperatorType};

pub fn build_initial_solution(
    instance: &Instance,
    data: &StopData,
) -> StopSolution {
    let mut sol = StopSolution::new(instance);
    let mut rng = thread_rng();

    repair(
        &mut sol,
        InsertionOperatorType::ProfitBased,
        instance,
        data,
        &mut rng,
    );

    sol
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_build_initial_solution() {
        let instance = Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 1.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 50.0, node_ids: vec![1], ..Default::default() },
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

        let data = StopData::new(&instance);
        let sol = build_initial_solution(&instance, &data);

        assert_eq!(sol.total_score, 50.0);
    }
}
