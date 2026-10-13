pub mod callback;
pub mod constraint;
pub mod ilp;
pub mod objective;
pub mod parser;
pub mod preprocess;
pub mod variable;

use ilp::Ilp;
use crate::common::{
    error::{SolverError, SolverErrorKind},
    instance::Instance,
    solution::Solution,
};
use crate::solvers::clutop::CluTopOptions;

pub fn solve<'a>(
    instance: &'a Instance,
    options: &CluTopOptions,
) -> Result<Solution<'a>, SolverError> {
    let ilp = Ilp::new(instance).map_err(|e| {
        SolverError::new(
            SolverErrorKind::Solver,
            &format!("Failed to build CluTOP Gurobi model: {}", e),
        )
    })?;

    ilp.solve(options)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    #[test]
    fn test_exact_solve_entrypoint() {
        let instance = Instance {
            name: "clutop_exact_test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 4.0, y: 0.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 50.0, node_ids: vec![1, 2], ..Default::default() },
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
            mode: SolverMode::Exact,
            time_limit: Some(5),
            max_iterations: 10,
            gurobi_params_file: None,
        };

        let sol = solve(&instance, &options).expect("Exact solve should succeed");
        assert_eq!(sol.total_score, 50.0);
    }
}
