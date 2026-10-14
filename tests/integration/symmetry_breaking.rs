use std::path::{Path, PathBuf};
use grb::prelude::*;
use tcops::{
    cli::{Cli, LibraryType, SolverMode},
    common::instance::{Cluster, Instance, Metric, Node, Point3, Subgroup, Vehicle},
    parser::load_instance,
    solvers::exact::gurobi::{constraint::symmetry_breaking, ilp::Ilp},
};

fn create_test_cli(input_path: &str) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
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
        enable_fractional_cuts: false,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
    }
}

/// Helper to create a small toy instance with specified vehicles
fn create_toy_instance(vehicles: Vec<Vehicle>) -> Instance {
    Instance {
        name: "toy_symmetry_instance".to_string(),
        metric: Metric::Euc2d,
        nodes: vec![
            Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
            Node { id: 1, point: Point3 { x: 5.0, y: 0.0, z: 0.0 }, ..Default::default() },
            Node { id: 2, point: Point3 { x: 0.0, y: 5.0, z: 0.0 }, ..Default::default() },
            Node { id: 3, point: Point3 { x: -5.0, y: 0.0, z: 0.0 }, ..Default::default() },
        ],
        subgroups: vec![
            Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
            Subgroup { id: 1, profit: 10.0, node_ids: vec![1], ..Default::default() },
            Subgroup { id: 2, profit: 10.0, node_ids: vec![2], ..Default::default() },
            Subgroup { id: 3, profit: 10.0, node_ids: vec![3], ..Default::default() },
        ],
        clusters: vec![
            Cluster { id: 0, subgroup_ids: vec![0] },
            Cluster { id: 1, subgroup_ids: vec![1] },
            Cluster { id: 2, subgroup_ids: vec![2] },
            Cluster { id: 3, subgroup_ids: vec![3] },
        ],
        vehicles,
        ..Default::default()
    }
}

#[test]
fn test_tcops_symmetry_breaking_preserves_optimality() -> grb::Result<()> {
    let instance_path = "resource/test/instance_07.tcops";
    if !Path::new(instance_path).exists() {
        eprintln!("Skipping test: instance file not found");
        return Ok(());
    }

    let mut instance = load_instance(Path::new(instance_path))
        .expect("Failed to load instance_07.tcops");

    // Make the fleet homogeneous with 2 identical vehicles
    instance.vehicles[0].budget = 110.0;
    instance.vehicles[1].budget = 110.0;
    instance.vehicles[0].start_node_id = 0;
    instance.vehicles[0].end_node_id = 0;
    instance.vehicles[1].start_node_id = 0;
    instance.vehicles[1].end_node_id = 0;

    let cli = create_test_cli(instance_path);

    // 1. Solve with standard Ilp (which always includes symmetry breaking)
    let ilp_with = Ilp::new(&instance)?;
    let sol_with = ilp_with.solve(&cli).expect("ILP with symmetry breaking should solve");

    // 2. Solve without symmetry breaking (manual model build omitting symmetry_breaking)
    let env = Env::new("gurobi.log")?;
    let mut model = Model::with_env("test_without_symm", &env)?;
    model.set_param(param::LazyConstraints, 1)?;
    model.set_param(param::Seed, 137)?;
    if let Some(tl) = cli.time_limit {
        model.set_param(param::TimeLimit, tl as f64)?;
    }

    let x = tcops::solvers::exact::gurobi::variable::initialize_x(&mut model, &instance)?;
    let y = tcops::solvers::exact::gurobi::variable::initialize_y(&mut model, &instance)?;
    let z = tcops::solvers::exact::gurobi::variable::initialize_z(&mut model, &instance)?;
    let w = tcops::solvers::exact::gurobi::variable::initialize_w(&mut model, &instance)?;
    let vars = tcops::solvers::exact::gurobi::ilp::DecisionVariables { x, y, z, w };

    tcops::solvers::exact::gurobi::constraint::intermediate_flow_conservation(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::unique_visit(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::depot_flow(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::depot_blocks(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::logical_physical_z_leq_y(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::logical_physical_y_leq_z(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::cluster(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::budget(&mut model, &vars, &instance)?;
    tcops::solvers::exact::gurobi::constraint::logical_visit(&mut model, &vars, &instance)?;
    // Notice: symmetry_breaking is intentionally omitted here to test parity!
    tcops::solvers::exact::gurobi::objective::set_objective(&mut model, &vars, &instance)?;

    let mut callback = tcops::solvers::exact::gurobi::callback::subtour::SubtourCallback::new(
        &vars,
        &instance,
        cli.enable_fractional_cuts,
    );
    model.optimize_with_callback(&mut callback)?;

    let z_without = model.get_attr(attr::ObjVal)?;

    assert_eq!(
        sol_with.total_score,
        z_without,
        "Optimal objective z* must be strictly identical with and without symmetry breaking"
    );

    Ok(())
}

#[test]
fn test_tcops_symmetry_breaking_orders_vehicles() -> grb::Result<()> {
    let instance_path = "resource/test/instance_07.tcops";
    if !Path::new(instance_path).exists() {
        eprintln!("Skipping test: instance file not found");
        return Ok(());
    }

    let mut instance = load_instance(Path::new(instance_path))
        .expect("Failed to load instance_07.tcops");

    // Make the fleet homogeneous with 2 identical vehicles
    instance.vehicles[0].budget = 110.0;
    instance.vehicles[1].budget = 110.0;
    instance.vehicles[0].start_node_id = 0;
    instance.vehicles[0].end_node_id = 0;
    instance.vehicles[1].start_node_id = 0;
    instance.vehicles[1].end_node_id = 0;

    let cli = create_test_cli(instance_path);

    let ilp = Ilp::new(&instance)?;
    let sol = ilp.solve(&cli).expect("ILP with symmetry breaking should solve");

    // Inspect routes of vehicle 0 and vehicle 1
    let custs_0: Vec<usize> = sol.routes[0].path.iter().copied().filter(|&n| n != 0).collect();
    let custs_1: Vec<usize> = sol.routes[1].path.iter().copied().filter(|&n| n != 0).collect();

    // If both vehicles visit customers, vehicle 0's minimum customer must be strictly smaller
    if !custs_1.is_empty() {
        assert!(
            !custs_0.is_empty(),
            "Vehicle 0 must be active if Vehicle 1 is active (hierarchical usage)"
        );
        let min_0 = custs_0.iter().min().unwrap();
        let min_1 = custs_1.iter().min().unwrap();
        assert!(
            min_0 < min_1,
            "Symmetry breaking must order vehicles: min customer of v0 ({}) must be < min customer of v1 ({})",
            min_0,
            min_1
        );

        // Also check the exact cut condition: every customer i in v1 requires a customer j < i in v0
        for &i in &custs_1 {
            let has_smaller_in_v0 = custs_0.iter().any(|&j| j < i);
            assert!(
                has_smaller_in_v0,
                "For node {} in v1, there must exist a node j < {} in v0",
                i, i
            );
        }
    }

    Ok(())
}

#[test]
fn test_tcops_symmetry_breaking_disabled_on_heterogeneous_fleet() -> grb::Result<()> {
    // 1. Heterogeneous by budget: Tmax_1 != Tmax_2
    let instance_diff_budget = create_toy_instance(vec![
        Vehicle { id: 0, budget: 30.0, start_node_id: 0, end_node_id: 0 },
        Vehicle { id: 1, budget: 50.0, start_node_id: 0, end_node_id: 0 },
    ]);

    let env = Env::new("gurobi.log")?;
    let mut model = Model::with_env("test_diff_budget", &env)?;
    let x = tcops::solvers::exact::gurobi::variable::initialize_x(&mut model, &instance_diff_budget)?;
    let y = tcops::solvers::exact::gurobi::variable::initialize_y(&mut model, &instance_diff_budget)?;
    let z = tcops::solvers::exact::gurobi::variable::initialize_z(&mut model, &instance_diff_budget)?;
    let w = tcops::solvers::exact::gurobi::variable::initialize_w(&mut model, &instance_diff_budget)?;
    let vars = tcops::solvers::exact::gurobi::ilp::DecisionVariables { x, y, z, w };

    symmetry_breaking(&mut model, &vars, &instance_diff_budget)?;
    model.update()?;
    assert_eq!(
        model.get_attr(attr::NumConstrs)?,
        0,
        "No symmetry constraints should be added when budgets differ"
    );

    // 2. Heterogeneous by start depot
    let instance_diff_start = create_toy_instance(vec![
        Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
        Vehicle { id: 1, budget: 50.0, start_node_id: 1, end_node_id: 0 },
    ]);

    let mut model2 = Model::with_env("test_diff_start", &env)?;
    let x2 = tcops::solvers::exact::gurobi::variable::initialize_x(&mut model2, &instance_diff_start)?;
    let y2 = tcops::solvers::exact::gurobi::variable::initialize_y(&mut model2, &instance_diff_start)?;
    let z2 = tcops::solvers::exact::gurobi::variable::initialize_z(&mut model2, &instance_diff_start)?;
    let w2 = tcops::solvers::exact::gurobi::variable::initialize_w(&mut model2, &instance_diff_start)?;
    let vars2 = tcops::solvers::exact::gurobi::ilp::DecisionVariables { x: x2, y: y2, z: z2, w: w2 };

    symmetry_breaking(&mut model2, &vars2, &instance_diff_start)?;
    model2.update()?;
    assert_eq!(
        model2.get_attr(attr::NumConstrs)?,
        0,
        "No symmetry constraints should be added when start depots differ"
    );

    // 3. Heterogeneous by end depot
    let instance_diff_end = create_toy_instance(vec![
        Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
        Vehicle { id: 1, budget: 50.0, start_node_id: 0, end_node_id: 1 },
    ]);

    let mut model3 = Model::with_env("test_diff_end", &env)?;
    let x3 = tcops::solvers::exact::gurobi::variable::initialize_x(&mut model3, &instance_diff_end)?;
    let y3 = tcops::solvers::exact::gurobi::variable::initialize_y(&mut model3, &instance_diff_end)?;
    let z3 = tcops::solvers::exact::gurobi::variable::initialize_z(&mut model3, &instance_diff_end)?;
    let w3 = tcops::solvers::exact::gurobi::variable::initialize_w(&mut model3, &instance_diff_end)?;
    let vars3 = tcops::solvers::exact::gurobi::ilp::DecisionVariables { x: x3, y: y3, z: z3, w: w3 };

    symmetry_breaking(&mut model3, &vars3, &instance_diff_end)?;
    model3.update()?;
    assert_eq!(
        model3.get_attr(attr::NumConstrs)?,
        0,
        "No symmetry constraints should be added when end depots differ"
    );

    Ok(())
}
