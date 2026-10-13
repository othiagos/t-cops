pub mod alns;
pub mod giant_tour;
pub mod local_search;
pub mod operators;
pub mod split;

use std::time::Instant;

use crate::common::{
    error::SolverError,
    instance::Instance,
    solution::{Route, Solution, SolutionStatus},
};
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::CluTopOptions;

pub fn solve<'a>(
    instance: &'a Instance,
    options: &CluTopOptions,
) -> Result<Solution<'a>, SolverError> {
    let start_time = Instant::now();
    let data = CluTopData::new(instance);

    let state = alns::solve_alns(instance, &data, options);

    let mut routes = Vec::new();
    for (r, path) in state.routes.into_iter().enumerate() {
        if path.len() > 2 {
            let cost = state.route_costs[r];
            routes.push(Route {
                vehicle_id: r,
                path,
                cost,
            });
        }
    }

    Ok(Solution {
        instance,
        duration: start_time.elapsed(),
        routes,
        total_cost: state.total_cost,
        total_score: state.total_score,
        status: SolutionStatus::Feasible,
        solver: Some("ALNS + Split (CluTOP)".to_string()),
        best_bound: None,
        gap: None,
        explored_nodes: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_heuristic_mod_solve() {
        let instance = Instance {
            name: "clutop_heur_test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 3.0, y: 4.0, z: 0.0 }, ..Default::default() },
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

        let options = CluTopOptions {
            mode: SolverMode::Heuristic,
            time_limit: Some(2),
            max_iterations: 10,
            gurobi_params_file: None,
        };

        let sol = solve(&instance, &options).expect("Heuristic solve should succeed");
        assert_eq!(sol.total_score, 50.0);
        assert_eq!(sol.routes.len(), 1);
        assert_eq!(sol.routes[0].path, vec![0, 1, 0]);
    }
}
