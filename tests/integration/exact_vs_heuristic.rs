use std::io::Write;
use std::path::PathBuf;
use tcops::{
    cli::{Cli, LibraryType, SolverMode},
    common::solution::SolutionStatus,
    parser::load_instance,
    solvers::{exact, heuristic},
};
use tempfile::NamedTempFile;

fn create_exact_cli(input_path: &str) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode: SolverMode::Exact,
        library: Some(LibraryType::Gurobi),
        max_iterations: 100,
        max_shaking_intensity: 20,
        show: false,
        save: false,
        time_limit: Some(30),
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
    }
}

fn create_heuristic_cli(input_path: &str) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode: SolverMode::Heuristic,
        library: None,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
        max_iterations: 200,
        max_shaking_intensity: 20,
        show: false,
        save: false,
        time_limit: None,
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
    }
}

/// Evaluates Heuristic score against Gurobi Exact Optimal score on burma14.tcops
#[test]
fn test_heuristic_matches_exact_burma14() {
    let instance_path = "resource/tcops/3/burma14.tcops";
    let instance = load_instance(std::path::Path::new(instance_path))
        .expect("Failed to load burma14.tcops");

    let exact_cli = create_exact_cli(instance_path);
    let exact_sol = exact::gurobi::solve(&instance, &exact_cli)
        .expect("Gurobi solve failed");
    assert_eq!(exact_sol.status, SolutionStatus::Optimal);

    let heur_cli = create_heuristic_cli(instance_path);
    let heur_sol = heuristic::solve(&instance, &heur_cli)
        .expect("Heuristic solve failed");

    println!(
        "Instance burma14 | Exact Score: {:.2} | Heuristic Score: {:.2}",
        exact_sol.total_score, heur_sol.total_score
    );

    assert!(
        heur_sol.total_score <= exact_sol.total_score + 1e-4,
        "Heuristic score ({}) cannot be greater than Exact optimal score ({})",
        heur_sol.total_score,
        exact_sol.total_score
    );

    let gap = (exact_sol.total_score - heur_sol.total_score) / exact_sol.total_score;
    assert!(
        gap <= 0.05,
        "Heuristic relative gap ({:.2}%) is too large compared to exact solution",
        gap * 100.0
    );
}

/// Evaluates Heuristic score against Gurobi Exact Optimal score on ulysses16.tcops
#[test]
fn test_heuristic_matches_exact_ulysses16() {
    let instance_path = "resource/tcops/3/ulysses16.tcops";
    let instance = load_instance(std::path::Path::new(instance_path))
        .expect("Failed to load ulysses16.tcops");

    let exact_cli = create_exact_cli(instance_path);
    let exact_sol = exact::gurobi::solve(&instance, &exact_cli)
        .expect("Gurobi solve failed");
    assert_eq!(exact_sol.status, SolutionStatus::Optimal);

    let heur_cli = create_heuristic_cli(instance_path);
    let heur_sol = heuristic::solve(&instance, &heur_cli)
        .expect("Heuristic solve failed");

    println!(
        "Instance ulysses16 | Exact Score: {:.2} | Heuristic Score: {:.2}",
        exact_sol.total_score, heur_sol.total_score
    );

    assert!(
        heur_sol.total_score <= exact_sol.total_score + 1e-4,
        "Heuristic score ({}) cannot be greater than Exact optimal score ({})",
        heur_sol.total_score,
        exact_sol.total_score
    );

    let gap = (exact_sol.total_score - heur_sol.total_score) / exact_sol.total_score;
    assert!(
        gap <= 0.05,
        "Heuristic relative gap ({:.2}%) is too large compared to exact solution",
        gap * 100.0
    );
}

/// Evaluates Heuristic score against Gurobi Exact Optimal score on a custom synthetic hierarchy instance
#[test]
fn test_heuristic_matches_exact_synthetic_hierarchy() {
    let content = "NAME: test_synth_comparison
TYPE: TCOPS
COMMENT: Synthetic comparison test
DIMENSION: 4
SUBGROUPS: 4
CLUSTERS: 3
VEHICLES: 1
EDGE_WEIGHT_TYPE: EUC_2D
NODE_COORD_SECTION: id profit x y
0 0.0 0.0 0.0
1 0.0 0.0 3.0
2 0.0 4.0 0.0
3 0.0 4.0 3.0
SUBGROUP_SECTION: subgroup_id profit id-vertex-list
0 10.0 1
1 20.0 2
2 30.0 3
3 0.0 0
CLUSTER_SECTION: cluster_id id-subgroup-list
0 0 1
1 2
2 3
VEHICLES_SECTION: id tmax start_node_id end_node_id
0 50.0 0 0
";
    let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
    temp_file.write_all(content.as_bytes()).unwrap();

    let instance = load_instance(temp_file.path()).unwrap();
    let exact_cli = create_exact_cli(temp_file.path().to_str().unwrap());
    let heur_cli = create_heuristic_cli(temp_file.path().to_str().unwrap());

    let exact_sol = exact::gurobi::solve(&instance, &exact_cli).unwrap();
    let heur_sol = heuristic::solve(&instance, &heur_cli).unwrap();

    assert_eq!(exact_sol.status, SolutionStatus::Optimal);
    assert_eq!(
        heur_sol.total_score, exact_sol.total_score,
        "Heuristic score ({}) should match exact optimal score ({}) on small synthetic instance",
        heur_sol.total_score, exact_sol.total_score
    );
}

/// Evaluates Heuristic score against Gurobi Exact Optimal score on a synthetic instance with distinct origin and destination
#[test]
fn test_heuristic_matches_exact_different_origin_destination() {
    let content = "NAME: test_diff_orig_dest_comparison
TYPE: TCOPS
COMMENT: Synthetic comparison test with distinct origin and destination
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

    let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
    temp_file.write_all(content.as_bytes()).unwrap();

    let instance = load_instance(temp_file.path()).unwrap();
    let exact_cli = create_exact_cli(temp_file.path().to_str().unwrap());
    let heur_cli = create_heuristic_cli(temp_file.path().to_str().unwrap());

    let exact_sol = exact::gurobi::solve(&instance, &exact_cli).unwrap();
    let heur_sol = heuristic::solve(&instance, &heur_cli).unwrap();

    assert_eq!(exact_sol.status, SolutionStatus::Optimal);
    assert_eq!(exact_sol.routes[0].path.first(), Some(&0));
    assert_eq!(exact_sol.routes[0].path.last(), Some(&3));

    assert_eq!(heur_sol.routes[0].path.first(), Some(&0));
    assert_eq!(heur_sol.routes[0].path.last(), Some(&3));

    assert_eq!(
        heur_sol.total_score, exact_sol.total_score,
        "Heuristic score ({}) should match exact optimal score ({}) on instance with distinct origin and destination",
        heur_sol.total_score, exact_sol.total_score
    );
}

/// Evaluates Heuristic against Gurobi on instance with multiple vehicles, different endpoints,
/// where one vehicle is used and another is unused.
#[test]
fn test_heuristic_matches_exact_one_used_one_unused_different_endpoints() {
    let content = "NAME: test_comp_used_unused
TYPE: TCOPS
COMMENT: Multi vehicle one used one unused distinct endpoints comparison
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
    let mut temp_file = NamedTempFile::new().expect("Failed to create temp file");
    temp_file.write_all(content.as_bytes()).unwrap();

    let instance = load_instance(temp_file.path()).unwrap();
    let exact_cli = create_exact_cli(temp_file.path().to_str().unwrap());
    let heur_cli = create_heuristic_cli(temp_file.path().to_str().unwrap());

    let exact_sol = exact::gurobi::solve(&instance, &exact_cli).unwrap();
    let heur_sol = heuristic::solve(&instance, &heur_cli).unwrap();

    assert_eq!(exact_sol.status, SolutionStatus::Optimal);
    assert_eq!(heur_sol.status, SolutionStatus::Feasible);

    // Exact: Vehicle 0 visits Node 1, Vehicle 1 is unused (path [3])
    assert!(exact_sol.routes[0].path.contains(&1));
    assert_eq!(exact_sol.routes[1].path, vec![3]);

    // Heuristic: Vehicle 0 visits Node 1, Vehicle 1 is unused (path [3])
    assert!(heur_sol.routes[0].path.contains(&1));
    assert_eq!(heur_sol.routes[1].path, vec![3]);

    assert_eq!(exact_sol.total_score, 10.0);
    assert_eq!(heur_sol.total_score, 10.0);
}
