use std::io::Write;
use std::path::PathBuf;
use tempfile::NamedTempFile;
use tcops::{
    cli::{Cli, SolverMode},
    common::solution::SolutionStatus,
    parser::load_instance,
    solvers::heuristic::solve,
};

fn create_heuristic_cli(input_path: &str, max_iterations: usize, max_shaking: usize) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode: SolverMode::Heuristic,
        library: None,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
        max_iterations,
        max_shaking_intensity: max_shaking,
        show: false,
        save: false,
        time_limit: None,
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
    }
}

fn load_test_instance(content: &str) -> (tcops::common::instance::Instance, NamedTempFile) {
    let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
    temp_file
        .write_all(content.as_bytes())
        .expect("Failed to write to temp file");
    let instance = load_instance(temp_file.path()).expect("Failed to parse instance");
    (instance, temp_file)
}

/// Baseline standard hierarchy test
#[test]
fn test_heuristic_baseline_single_hierarchy() {
    let content = "NAME: test_heur_baseline
TYPE: TCOPS
COMMENT: Heuristic baseline test
DIMENSION: 3
SUBGROUPS: 3
CLUSTERS: 3
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 0.0 3.0
2 0.0 4.0 0.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 20.0 2
2 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert_eq!(solution.total_score, 30.0);
    assert!(solution.total_cost <= 50.0);
    assert!(!solution.routes.is_empty());
    assert_eq!(solution.routes[0].path.first(), Some(&0));
    assert_eq!(solution.routes[0].path.last(), Some(&0));
}

/// Node in multiple subgroups (Node 1 in Subgroups 0 and 1)
#[test]
fn test_heuristic_node_in_multiple_subgroups() {
    let content = "NAME: test_heur_node_multi
TYPE: TCOPS
COMMENT: Node in multiple subgroups heuristic test
DIMENSION: 3
SUBGROUPS: 4
CLUSTERS: 4
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 3.0 4.0
2 0.0 10.0 10.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 15.0 1
1 25.0 1
2 0.0 0
3 0.0 2
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert!(solution.total_score > 0.0);
    assert!(solution.total_cost <= 50.0);
    assert!(solution.routes[0].path.contains(&1));
}

/// Subgroup in multiple clusters (Subgroup 1 in Clusters 0 and 1)
#[test]
fn test_heuristic_subgroup_in_multiple_clusters() {
    let content = "NAME: test_heur_subgroup_multi
TYPE: TCOPS
COMMENT: Subgroup in multiple clusters heuristic test
DIMENSION: 4
SUBGROUPS: 4
CLUSTERS: 3
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 1.0 0.0
2 0.0 0.0 1.0
3 0.0 1.0 1.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 30.0 2
2 20.0 3
3 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0 1
1 1 2
2 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert!(solution.total_score > 0.0);
    assert!(solution.total_cost <= 50.0);
}

/// Combined multi-node and multi-cluster hierarchy
#[test]
fn test_heuristic_combined_multi_hierarchy() {
    let content = "NAME: test_heur_combined
TYPE: TCOPS
COMMENT: Combined hierarchy heuristic test
DIMENSION: 4
SUBGROUPS: 4
CLUSTERS: 3
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 2.0 0.0
2 0.0 0.0 2.0
3 0.0 2.0 2.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 15.0 1
1 25.0 1 2
2 35.0 2 3
3 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0 1
1 1 2
2 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert!(solution.total_score > 0.0);
    assert!(solution.total_cost <= 50.0);
}

/// Tight budget constraint - cannot visit all nodes
#[test]
fn test_heuristic_tight_budget_constraint() {
    let content = "NAME: test_heur_tight_budget
TYPE: TCOPS
COMMENT: Tight budget constraint test
DIMENSION: 4
SUBGROUPS: 4
CLUSTERS: 4
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 1.0 0.0
2 0.0 10.0 0.0
3 0.0 20.0 0.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 5.0 1
1 15.0 2
2 25.0 3
3 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 5.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 10, 2);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert!(solution.total_cost <= 5.0);
    assert_eq!(solution.total_score, 5.0);
    assert!(solution.routes[0].path.contains(&1));
    assert!(!solution.routes[0].path.contains(&2));
    assert!(!solution.routes[0].path.contains(&3));
}

/// Multi-vehicle distribution
#[test]
fn test_heuristic_multi_vehicle_distribution() {
    let content = "NAME: test_heur_multi_vehicle
TYPE: TCOPS
COMMENT: Multi vehicle test
DIMENSION: 3
SUBGROUPS: 3
CLUSTERS: 3
VEHICLES: 2
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 0.0 3.0
2 0.0 4.0 0.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 20.0 2
2 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 10.0 0 0
1 10.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 15, 3);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert_eq!(solution.routes.len(), 2);
    assert_eq!(solution.total_score, 30.0);
}

/// Zero budget vehicle
#[test]
fn test_heuristic_zero_budget_vehicle() {
    let content = "NAME: test_heur_zero_budget
TYPE: TCOPS
COMMENT: Zero budget vehicle test
DIMENSION: 2
SUBGROUPS: 2
CLUSTERS: 2
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 1.0 1.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 0.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 5, 1);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.total_score, 0.0);
    assert_eq!(solution.total_cost, 0.0);
}

/// Heuristic solver test where origin (start_node_id) and destination (end_node_id) are different
#[test]
fn test_heuristic_different_origin_destination_single_vehicle() {
    let content = "NAME: test_heur_diff_origin_dest
TYPE: TCOPS
COMMENT: Heuristic distinct origin and destination test
DIMENSION: 4
SUBGROUPS: 4
CLUSTERS: 4
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id x y
0 0.0 0.0
1 0.0 3.0
2 4.0 0.0
3 4.0 3.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 20.0 2
2 0.0 0
3 0.0 3
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 3
";

    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed for different origin and destination");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert!(!solution.routes.is_empty());

    let route = &solution.routes[0];
    assert_eq!(route.path.first(), Some(&0), "Route must start at origin (0)");
    assert_eq!(route.path.last(), Some(&3), "Route must end at destination (3)");
    assert!(route.path.first() != route.path.last(), "Origin and destination must be different");
    assert_eq!(solution.total_score, 30.0);
}

/// Heuristic solver test with multi-vehicle where each vehicle has different origin and destination
#[test]
fn test_heuristic_different_origin_destination_multi_vehicle() {
    let content = "NAME: test_heur_multi_diff_origin_dest
TYPE: TCOPS
COMMENT: Heuristic multi vehicle distinct origin and destination test
DIMENSION: 6
SUBGROUPS: 6
CLUSTERS: 6
VEHICLES: 2
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id x y
0 0.0 0.0
1 0.0 3.0
2 4.0 3.0
3 10.0 0.0
4 10.0 3.0
5 14.0 3.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 20.0 4
2 0.0 0
3 0.0 2
4 0.0 3
5 0.0 5
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
4 4
5 5
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 10.0 0 2
1 10.0 3 5
";

    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed for multi-vehicle distinct endpoints");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert_eq!(solution.routes.len(), 2);

    let route0 = &solution.routes[0];
    assert_eq!(route0.path.first(), Some(&0), "Vehicle 0 route must start at 0");
    assert_eq!(route0.path.last(), Some(&2), "Vehicle 0 route must end at 2");

    let route1 = &solution.routes[1];
    assert_eq!(route1.path.first(), Some(&3), "Vehicle 1 route must start at 3");
    assert_eq!(route1.path.last(), Some(&5), "Vehicle 1 route must end at 5");

    assert_eq!(solution.total_score, 30.0);
}

/// Heuristic test for multi-vehicle instance where vehicles have distinct origins and destinations,
/// and Vehicle 0 is used while Vehicle 1 remains unused.
#[test]
fn test_heuristic_multi_vehicle_one_used_one_unused_different_endpoints() {
    let content = "NAME: test_heur_multi_used_unused
TYPE: TCOPS
COMMENT: Heuristic multi vehicle one used one unused distinct endpoints
DIMENSION: 6
SUBGROUPS: 6
CLUSTERS: 6
VEHICLES: 2
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id x y
0 0.0 0.0
1 0.0 3.0
2 4.0 3.0
3 10.0 0.0
4 10.0 3.0
5 14.0 3.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 0.0 0
2 0.0 2
3 0.0 3
4 0.0 4
5 0.0 5
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
4 4
5 5
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 20.0 0 2
1 20.0 3 5
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_heuristic_cli(temp_file.path().to_str().unwrap(), 20, 5);

    let solution = solve(&instance, &cli).expect("Heuristic solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Feasible);
    assert_eq!(solution.routes.len(), 2);

    // Vehicle 0 is used: visits Node 1
    let route0 = &solution.routes[0];
    assert_eq!(route0.path.first(), Some(&0), "Vehicle 0 starts at 0");
    assert!(route0.path.contains(&1), "Vehicle 0 visits node 1");
    assert_eq!(route0.path.last(), Some(&2), "Vehicle 0 ends at 2");

    // Vehicle 1 is unused: route path is truncated to [3] with cost 0.0
    let route1 = &solution.routes[1];
    assert_eq!(route1.path.first(), Some(&3), "Vehicle 1 starts at 3");
    assert_eq!(route1.path, vec![3], "Unused Vehicle 1 path should be [3]");

    assert_eq!(solution.total_score, 10.0);
}





