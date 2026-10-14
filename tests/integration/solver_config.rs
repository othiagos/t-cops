use std::collections::HashSet;
use std::path::{Path, PathBuf};
use grb::prelude::*;
use tcops::{
    cli::{Cli, LibraryType, SolverMode},
    common::instance::{Cluster, Instance, Metric, Node, Point3, Subgroup, Vehicle},
    parser::load_instance,
    solvers::{
        clutop::{self, exact::ilp::Ilp as CluTopIlp, CluTopOptions},
        exact::{self, gurobi::ilp::Ilp as TcopsIlp},
        stop::{self, exact::ilp::Ilp as StopIlp, StopOptions},
    },
};

fn create_test_instance() -> Instance {
    Instance {
        name: "test_config_instance".to_string(),
        metric: Metric::Euc2d,
        nodes: vec![
            Node {
                id: 0,
                point: Point3 { x: 0.0, y: 0.0, z: 0.0 },
                parent_subgroup_ids: HashSet::new(),
            },
            Node {
                id: 1,
                point: Point3 { x: 0.0, y: 3.0, z: 0.0 },
                parent_subgroup_ids: HashSet::from([0]),
            },
        ],
        subgroups: vec![
            Subgroup {
                id: 0,
                profit: 10.0,
                node_ids: vec![1],
                parent_cluster_ids: HashSet::from([0]),
            },
        ],
        clusters: vec![
            Cluster {
                id: 0,
                subgroup_ids: vec![0],
            },
        ],
        vehicles: vec![
            Vehicle {
                id: 0,
                budget: 20.0,
                start_node_id: 0,
                end_node_id: 0,
            },
        ],
        ..Default::default()
    }
}

fn create_test_cli() -> Cli {
    Cli {
        input: PathBuf::from("dummy.tcops"),
        mode: SolverMode::Exact,
        library: Some(LibraryType::Gurobi),
        max_iterations: 10,
        max_shaking_intensity: 5,
        show: false,
        save: false,
        time_limit: Some(10),
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
    }
}

#[test]
fn test_solver_parameters_seed_and_threads() -> grb::Result<()> {
    let instance = create_test_instance();
    let expected_threads = std::thread::available_parallelism()
        .map(|n| n.get() as i32)
        .unwrap_or(1);

    // 1. T-COPS Ilp
    let cli = create_test_cli();
    let mut tcops_ilp = TcopsIlp::new(&instance)?;
    tcops_ilp.configure_solver(&cli).expect("T-COPS configure_solver should succeed");
    let model = &tcops_ilp.model;
    assert_eq!(model.get_param(param::Seed)?, 137, "T-COPS Seed must be 137");
    assert_eq!(
        model.get_param(param::Threads)?,
        expected_threads,
        "T-COPS Threads must match available_parallelism"
    );

    // 2. CluTOP Ilp
    let clutop_options = CluTopOptions {
        mode: SolverMode::Exact,
        time_limit: Some(10),
        max_iterations: 10,
        gurobi_params_file: None,
    };
    let mut clutop_ilp = CluTopIlp::new(&instance)?;
    clutop_ilp
        .configure_solver(&clutop_options)
        .expect("CluTOP configure_solver should succeed");
    let model = &clutop_ilp.model;
    assert_eq!(model.get_param(param::Seed)?, 137, "CluTOP Seed must be 137");
    assert_eq!(
        model.get_param(param::Threads)?,
        expected_threads,
        "CluTOP Threads must match available_parallelism"
    );

    // 3. STOP Ilp
    let stop_options = StopOptions {
        mode: SolverMode::Exact,
        time_limit: Some(10),
        max_iterations: 10,
        gurobi_params_file: None,
    };
    let mut stop_ilp = StopIlp::new(&instance)?;
    stop_ilp
        .configure_solver(&stop_options)
        .expect("STOP configure_solver should succeed");
    let model = &stop_ilp.model;
    assert_eq!(model.get_param(param::Seed)?, 137, "STOP Seed must be 137");
    assert_eq!(
        model.get_param(param::Threads)?,
        expected_threads,
        "STOP Threads must match available_parallelism"
    );

    Ok(())
}

#[test]
fn test_solver_parameters_file_precedence() -> grb::Result<()> {
    use std::io::Write;
    use tempfile::NamedTempFile;

    let instance = create_test_instance();
    let mut param_file = NamedTempFile::new().expect("Failed to create temp param file");
    writeln!(param_file, "Threads 2").expect("Failed to write param file");

    let mut cli = create_test_cli();
    cli.gurobi_params_file = Some(param_file.path().to_str().unwrap().to_string());

    let mut tcops_ilp = TcopsIlp::new(&instance)?;
    tcops_ilp.configure_solver(&cli).expect("T-COPS configure_solver should succeed");

    // Seed was not overridden by file, so base default 137 remains
    assert_eq!(tcops_ilp.model.get_param(param::Seed)?, 137);
    // Threads was overridden to 2 by params file
    assert_eq!(tcops_ilp.model.get_param(param::Threads)?, 2);

    Ok(())
}

#[test]
fn test_deterministic_reproducibility() -> grb::Result<()> {
    let instance_path = "resource/tcops/1/burma14.tcops";
    if !Path::new(instance_path).exists() {
        eprintln!("Skipping test_deterministic_reproducibility: instance file not found");
        return Ok(());
    }

    let instance = load_instance(Path::new(instance_path))
        .expect("Failed to load burma14.tcops");

    // 1. T-COPS deterministic reproducibility test
    let cli = Cli {
        input: PathBuf::from(instance_path),
        mode: SolverMode::Exact,
        library: Some(LibraryType::Gurobi),
        max_iterations: 10,
        max_shaking_intensity: 5,
        show: false,
        save: false,
        time_limit: Some(15),
        folder_result: "./result".to_string(),
        custom_result_name: None,
        gurobi_params_file: None,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
    };

    let sol1 = exact::gurobi::solve(&instance, &cli).expect("T-COPS run 1 failed");
    let sol2 = exact::gurobi::solve(&instance, &cli).expect("T-COPS run 2 failed");

    assert_eq!(sol1.total_score, sol2.total_score, "T-COPS total_score (z*) must be identical");
    assert_eq!(sol1.explored_nodes, sol2.explored_nodes, "T-COPS explored_nodes must be identical");
    assert_eq!(sol1.best_bound, sol2.best_bound, "T-COPS best_bound must be identical");
    assert_eq!(sol1.gap, sol2.gap, "T-COPS gap must be identical");
    assert_eq!(sol1.routes.len(), sol2.routes.len(), "T-COPS routes count must be identical");
    for (r1, r2) in sol1.routes.iter().zip(sol2.routes.iter()) {
        assert_eq!(r1.vehicle_id, r2.vehicle_id, "Route vehicle_id must match");
        assert_eq!(r1.path, r2.path, "Route path must match");
        assert_eq!(r1.cost, r2.cost, "Route cost must match");
    }

    // 2. CluTOP deterministic reproducibility test
    let clutop_options = CluTopOptions {
        mode: SolverMode::Exact,
        time_limit: Some(15),
        max_iterations: 10,
        gurobi_params_file: None,
    };

    let clutop_sol1 = clutop::solve(&instance, &clutop_options).expect("CluTOP run 1 failed");
    let clutop_sol2 = clutop::solve(&instance, &clutop_options).expect("CluTOP run 2 failed");

    assert_eq!(clutop_sol1.total_score, clutop_sol2.total_score, "CluTOP total_score (z*) must be identical");
    assert_eq!(clutop_sol1.explored_nodes, clutop_sol2.explored_nodes, "CluTOP explored_nodes must be identical");
    for (r1, r2) in clutop_sol1.routes.iter().zip(clutop_sol2.routes.iter()) {
        assert_eq!(r1.vehicle_id, r2.vehicle_id);
        assert_eq!(r1.path, r2.path);
        assert_eq!(r1.cost, r2.cost);
    }

    // 3. STOP deterministic reproducibility test
    let stop_options = StopOptions {
        mode: SolverMode::Exact,
        time_limit: Some(15),
        max_iterations: 10,
        gurobi_params_file: None,
    };

    let stop_sol1 = stop::solve(&instance, &stop_options).expect("STOP run 1 failed");
    let stop_sol2 = stop::solve(&instance, &stop_options).expect("STOP run 2 failed");

    assert_eq!(stop_sol1.total_score, stop_sol2.total_score, "STOP total_score (z*) must be identical");
    assert_eq!(stop_sol1.explored_nodes, stop_sol2.explored_nodes, "STOP explored_nodes must be identical");
    for (r1, r2) in stop_sol1.routes.iter().zip(stop_sol2.routes.iter()) {
        assert_eq!(r1.vehicle_id, r2.vehicle_id);
        assert_eq!(r1.path, r2.path);
        assert_eq!(r1.cost, r2.cost);
    }

    Ok(())
}
