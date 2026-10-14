use std::path::PathBuf;
use tcops::{
    cli::{Cli, LibraryType, SolverMode},
    solvers::{exact, heuristic},
};

fn create_e2e_cli(
    input_path: &str,
    mode: SolverMode,
    library: Option<LibraryType>,
) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode,
        library,
        max_iterations: 50,
        max_shaking_intensity: 10,
        show: false,
        save: false,
        time_limit: Some(15),
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
    }
}

/// E2E Consistency test between Gurobi optimal score and Heuristic score
#[test]
fn test_e2e_gurobi_vs_heuristic_consistency() {
    let instance_path = "resource/tcops/3/burma14.tcops";

    let gurobi_cli = create_e2e_cli(instance_path, SolverMode::Exact, Some(LibraryType::Gurobi));
    let inst = tcops::parser::load_instance(std::path::Path::new(instance_path)).unwrap();
    let gurobi_sol = exact::gurobi::solve(&inst, &gurobi_cli).unwrap();

    let heur_cli = create_e2e_cli(instance_path, SolverMode::Heuristic, None);
    let heur_sol = heuristic::solve(&inst, &heur_cli).unwrap();

    // Both solvers return positive score
    assert!(gurobi_sol.total_score > 0.0);
    assert!(heur_sol.total_score > 0.0);

    // Heuristic total score cannot exceed Gurobi optimal score
    assert!(
        heur_sol.total_score <= gurobi_sol.total_score + 1e-4,
        "Heuristic score ({}) cannot exceed Gurobi optimal score ({})",
        heur_sol.total_score,
        gurobi_sol.total_score
    );

    // Both solvers produce valid routes respecting vehicle budget
    for route in &gurobi_sol.routes {
        let vehicle = &inst.vehicles[route.vehicle_id];
        assert!(route.cost <= vehicle.budget);
    }
    for route in &heur_sol.routes {
        let vehicle = &inst.vehicles[route.vehicle_id];
        assert!(route.cost <= vehicle.budget);
    }
}

/// E2E pipeline test for instance with distinct origin and destination (start_node_id != end_node_id)
#[test]
fn test_e2e_different_origin_destination_pipeline() {
    let content = "NAME: test_e2e_diff_endpoints
TYPE: TCOPS
COMMENT: E2E distinct endpoints test
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

    let temp_dir = tempfile::tempdir().expect("Failed to create temp dir");
    let input_path = temp_dir.path().join("instance.tcops");
    std::fs::write(&input_path, content).unwrap();

    let gurobi_cli = create_e2e_cli(input_path.to_str().unwrap(), SolverMode::Exact, Some(LibraryType::Gurobi));
    let inst = tcops::parser::load_instance(&input_path).unwrap();
    let gurobi_sol = exact::gurobi::solve(&inst, &gurobi_cli).unwrap();

    let heur_cli = create_e2e_cli(input_path.to_str().unwrap(), SolverMode::Heuristic, None);
    let heur_sol = heuristic::solve(&inst, &heur_cli).unwrap();

    assert_eq!(gurobi_sol.routes[0].path.first(), Some(&0));
    assert_eq!(gurobi_sol.routes[0].path.last(), Some(&3));

    assert_eq!(heur_sol.routes[0].path.first(), Some(&0));
    assert_eq!(heur_sol.routes[0].path.last(), Some(&3));

    assert_eq!(gurobi_sol.total_score, 30.0);
    assert_eq!(heur_sol.total_score, 30.0);
}
