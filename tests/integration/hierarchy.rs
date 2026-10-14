use std::io::Write;
use std::path::PathBuf;
use tcops::{
    cli::{Cli, LibraryType, SolverMode},
    common::solution::SolutionStatus,
    parser::load_instance,
    solvers::exact::{self, gurobi::solve},
};
use tempfile::NamedTempFile;

fn create_cli(input_path: &str) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode: SolverMode::Exact,
        library: Some(LibraryType::Gurobi),
        max_iterations: 100,
        max_shaking_intensity: 20,
        show: false,
        save: false,
        time_limit: Some(10),
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
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

/// Baseline Hierarchy - Node in 1 subgroup, Subgroup in 1 cluster
#[test]
fn test_gurobi_hierarchy_single_node_single_subgroup_single_cluster() {
    let content = "NAME: test_single_hierarchy
TYPE: TCOPS
COMMENT: Baseline hierarchy test
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

    assert_eq!(instance.nodes[1].parent_subgroup_ids.len(), 1);
    assert!(instance.nodes[1].parent_subgroup_ids.contains(&0));
    assert_eq!(instance.nodes[2].parent_subgroup_ids.len(), 1);
    assert!(instance.nodes[2].parent_subgroup_ids.contains(&1));

    assert_eq!(instance.clusters[0].subgroup_ids, vec![0]);
    assert_eq!(instance.clusters[1].subgroup_ids, vec![1]);

    let cli = create_cli(temp_file.path().to_str().unwrap());
    let solution = solve(&instance, &cli).expect("Gurobi direct solve failed");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.solver, Some("Gurobi".to_string()));
    assert!(!solution.routes.is_empty());

    let route = &solution.routes[0];
    assert_eq!(route.path.first(), Some(&0));
    assert_eq!(route.path.last(), Some(&0));
    assert!(solution.total_cost <= 50.0);
    assert_eq!(solution.total_score, 30.0);
}

/// Node in MULTIPLE subgroups (Node 1 belongs to Subgroup 0 AND Subgroup 1)
#[test]
fn test_gurobi_hierarchy_node_in_multiple_subgroups() {
    let content = "NAME: test_node_multi_subgroups
TYPE: TCOPS
COMMENT: Node in multiple subgroups test
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

    assert_eq!(instance.nodes[1].parent_subgroup_ids.len(), 2);
    assert!(instance.nodes[1].parent_subgroup_ids.contains(&0));
    assert!(instance.nodes[1].parent_subgroup_ids.contains(&1));

    let cli = create_cli(temp_file.path().to_str().unwrap());
    let solution =
        solve(&instance, &cli).expect("Gurobi solve failed for node in multiple subgroups");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.solver, Some("Gurobi".to_string()));
    assert!(!solution.routes.is_empty());

    let route = &solution.routes[0];
    assert_eq!(route.path.first(), Some(&0));
    assert_eq!(route.path.last(), Some(&0));
    assert!(route.path.contains(&1));
    assert!(solution.total_cost <= 50.0);
    assert!(solution.total_score > 0.0);
}

/// Test specifically exposing the bug in logical_physical constraint:
/// Node 1 belongs to Subgroup 0 (15.0) AND Subgroup 1 (25.0). Real profit = 40.0.
/// Node 2 belongs to Subgroup 2 (30.0). Real profit = 30.0.
/// Vehicle budget = 8.0 (can visit Node 1 OR Node 2, but not both).
/// Under buggy constraint (sum_z == sum_y), Gurobi caps Node 1 at 25.0 < 30.0,
/// choosing Node 2 (score 30.0) instead of the true optimal Node 1 (score 40.0).
#[test]
fn test_gurobi_node_in_multiple_subgroups_should_collect_all_subgroups() {
    let content = "NAME: test_expose_multi_subgroup_bug
TYPE: TCOPS
COMMENT: Exposing logical_physical bug for nodes in multiple subgroups
DIMENSION: 4
SUBGROUPS: 5
CLUSTERS: 5
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id x y
0 0.0 0.0
1 0.0 3.0
2 3.0 0.0
3 10.0 10.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 15.0 1
1 25.0 1
2 30.0 2
3 0.0 0
4 0.0 3
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
2 2
3 3
4 4
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 8.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_cli(temp_file.path().to_str().unwrap());
    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert!(
        solution.routes[0].path.contains(&1),
        "Optimal route must visit Node 1 (profit 40.0) instead of Node 2 (profit 30.0)"
    );
    assert_eq!(
        solution.total_score, 40.0,
        "Visiting Node 1 once should collect both Subgroup 0 (15.0) and Subgroup 1 (25.0) for a total score of 40.0"
    );
}




/// Subgroup in MULTIPLE clusters (Subgroup 1 belongs to Cluster 0 AND Cluster 1)
#[test]
fn test_gurobi_hierarchy_subgroup_in_multiple_clusters() {
    let content = "NAME: test_subgroup_multi_clusters
TYPE: TCOPS
COMMENT: Subgroup in multiple clusters test
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

    assert!(instance.clusters[0].subgroup_ids.contains(&1));
    assert!(instance.clusters[1].subgroup_ids.contains(&1));

    let cli = create_cli(temp_file.path().to_str().unwrap());
    let solution =
        solve(&instance, &cli).expect("Gurobi solve failed for subgroup in multiple clusters");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.solver, Some("Gurobi".to_string()));
    assert!(!solution.routes.is_empty());
    assert!(solution.total_score > 0.0);
    assert!(solution.total_cost <= 50.0);
}

/// Combined hierarchy (Node in MULTIPLE subgroups AND Subgroup in MULTIPLE clusters)
#[test]
fn test_gurobi_hierarchy_combined_multi_nodes_and_clusters() {
    let content = "NAME: test_combined_hierarchy
TYPE: TCOPS
COMMENT: Node in multiple subgroups AND Subgroup in multiple clusters
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

    assert!(instance.nodes[1].parent_subgroup_ids.contains(&0));
    assert!(instance.nodes[1].parent_subgroup_ids.contains(&1));
    assert!(instance.nodes[2].parent_subgroup_ids.contains(&1));
    assert!(instance.nodes[2].parent_subgroup_ids.contains(&2));
    assert!(instance.clusters[0].subgroup_ids.contains(&1));
    assert!(instance.clusters[1].subgroup_ids.contains(&1));

    let cli = create_cli(temp_file.path().to_str().unwrap());
    let solution = solve(&instance, &cli).expect("Gurobi solve failed for combined hierarchy");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.solver, Some("Gurobi".to_string()));
    assert!(!solution.routes.is_empty());
    assert!(solution.total_score > 0.0);
    assert!(solution.total_cost <= 50.0);
}

/// Exact solver routing test (using exact::solve dispatcher with Gurobi)
#[test]
fn test_gurobi_hierarchy_via_exact_solver_routing() {
    let content = "NAME: test_routing_hierarchy
TYPE: TCOPS
COMMENT: Test solver routing via exact::solve
DIMENSION: 3
SUBGROUPS: 3
CLUSTERS: 3
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 1.0 1.0
2 0.0 2.0 2.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 12.0 1
1 18.0 1 2
2 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0 1
1 1
2 2
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = exact::solve(&instance, &cli).expect("exact::solve with Gurobi library failed");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.solver, Some("Gurobi".to_string()));
    assert!(solution.total_score > 0.0);
}

/// Zero budget vehicle test
#[test]
fn test_gurobi_zero_budget_vehicle() {
    let content = "NAME: test_zero_budget
TYPE: TCOPS
COMMENT: Zero budget vehicle test
DIMENSION: 2
SUBGROUPS: 2
CLUSTERS: 2
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 3.0 4.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 50.0 1
1 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0
1 1
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 0.0 0 0
";
    let (instance, temp_file) = load_test_instance(content);
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed for 0 budget");
    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.total_score, 0.0);
    assert_eq!(solution.total_cost, 0.0);
}

/// Multi-vehicle routing
#[test]
fn test_gurobi_multi_vehicle_routes() {
    let content = "NAME: test_multi_vehicle
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
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed for multi-vehicle");
    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.total_score, 30.0);
}

/// Gurobi exact solver test where origin (start_node_id) and destination (end_node_id) are different
#[test]
fn test_gurobi_different_origin_destination_single_vehicle() {
    let content = "NAME: test_diff_origin_dest
TYPE: TCOPS
COMMENT: Distinct origin and destination test
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
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed for different origin and destination");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert!(!solution.routes.is_empty());

    let route = &solution.routes[0];
    assert_eq!(route.path.first(), Some(&0), "Route must start at origin (0)");
    assert_eq!(route.path.last(), Some(&3), "Route must end at destination (3)");
    assert!(route.path.first() != route.path.last(), "Origin and destination must be different");
    assert_eq!(solution.total_score, 30.0);
}

/// Gurobi exact solver test with multi-vehicle where each vehicle has different origin and destination
#[test]
fn test_gurobi_different_origin_destination_multi_vehicle() {
    let content = "NAME: test_multi_diff_origin_dest
TYPE: TCOPS
COMMENT: Multi vehicle distinct origin and destination test
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
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed for multi-vehicle distinct endpoints");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.routes.len(), 2);

    let route0 = &solution.routes[0];
    assert_eq!(route0.path.first(), Some(&0), "Vehicle 0 route must start at 0");
    assert_eq!(route0.path.last(), Some(&2), "Vehicle 0 route must end at 2");

    let route1 = &solution.routes[1];
    assert_eq!(route1.path.first(), Some(&3), "Vehicle 1 route must start at 3");
    assert_eq!(route1.path.last(), Some(&5), "Vehicle 1 route must end at 5");

    assert_eq!(solution.total_score, 30.0);
}

/// Test multi-vehicle instance where vehicles have distinct origins and destinations,
/// and Vehicle 0 is used (visits a subgroup) while Vehicle 1 remains unused.
#[test]
fn test_gurobi_multi_vehicle_one_used_one_unused_different_endpoints() {
    let content = "NAME: test_multi_used_unused
TYPE: TCOPS
COMMENT: Multi vehicle one used one unused distinct endpoints
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
    let cli = create_cli(temp_file.path().to_str().unwrap());

    let solution = solve(&instance, &cli).expect("Gurobi solve should succeed");

    assert_eq!(solution.status, SolutionStatus::Optimal);
    assert_eq!(solution.routes.len(), 2);

    // Vehicle 0 is used: visits Node 1
    let route0 = &solution.routes[0];
    assert_eq!(route0.path.first(), Some(&0), "Vehicle 0 starts at 0");
    assert!(route0.path.contains(&1), "Vehicle 0 visits node 1");
    assert_eq!(route0.path.last(), Some(&2), "Vehicle 0 ends at 2");

    // Vehicle 1 is unused: does not visit any intermediate customer nodes
    let route1 = &solution.routes[1];
    assert_eq!(route1.path.first(), Some(&3), "Vehicle 1 starts at 3");
    assert!(!route1.path.contains(&1), "Vehicle 1 does not visit node 1");
    assert!(!route1.path.contains(&4), "Vehicle 1 does not visit node 4");

    // Check that total score is only from Vehicle 0's subgroup (10.0)
    assert_eq!(solution.total_score, 10.0);
}




