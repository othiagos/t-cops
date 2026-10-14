use crate::{
    cli::Cli,
    common::{error::SolverError, instance::Instance, solution::Solution},
};

mod vns;

pub fn solve<'a>(instance: &'a Instance, args: &Cli) -> Result<Solution<'a>, SolverError> {
    vns::solve(instance, args.max_iterations, args.max_shaking_intensity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use std::path::PathBuf;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            name: "heuristic_mod_test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 3.0, y: 4.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 15.0, node_ids: vec![1], parent_cluster_ids: HashSet::from([0]) },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_heuristic_solve() {
        let instance = create_test_instance();
        let args = Cli {
            input: PathBuf::from("dummy.tcops"),
            mode: SolverMode::Heuristic,
            library: None,
            #[cfg(feature = "lib_good_lp")]
            solver: None,
            max_iterations: 10,
            max_shaking_intensity: 5,
            show: false,
            save: false,
            time_limit: None,
            folder_result: "./result".to_string(),
            custom_result_name: None,
            gurobi_params_file: None,
            enable_fractional_cuts: false,
        };

        let solution = solve(&instance, &args).expect("Heuristic solve should succeed");
        assert_eq!(solution.total_score, 15.0);
        assert_eq!(solution.routes[0].path, vec![0, 1, 0]);
    }
}
