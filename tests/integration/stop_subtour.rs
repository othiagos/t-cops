#![allow(clippy::useless_conversion)]

use std::collections::HashSet;
use grb::prelude::*;
use tcops::{
    cli::SolverMode,
    common::{
        instance::{Cluster, Instance, Metric, Node, Point3, Subgroup, Vehicle},
        solution::SolutionStatus,
    },
    solvers::stop::{
        self,
        cluster::StopData,
        exact::{
            callback::subtour::{build_cut_set_expr, build_dfj_expr, StopSubtourCallback},
            variable::initialize_variables,
        },
        StopOptions,
    },
};

#[test]
fn test_stop_subtour_cutset_generation() -> grb::Result<()> {
    let env = Env::new("gurobi.log")?;
    let mut model = Model::with_env("test_stop_cutset_gen", &env)?;

    let instance = Instance {
        name: "test_cutset_gen_instance".to_string(),
        metric: Metric::Euc2d,
        nodes: vec![
            Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
            Node { id: 1, point: Point3 { x: 10.0, y: 0.0, z: 0.0 }, ..Default::default() },
            Node { id: 2, point: Point3 { x: 10.0, y: 5.0, z: 0.0 }, ..Default::default() },
            Node { id: 3, point: Point3 { x: 20.0, y: 20.0, z: 0.0 }, ..Default::default() },
        ],
        subgroups: vec![
            Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
            Subgroup { id: 1, profit: 50.0, node_ids: vec![1, 2], ..Default::default() },
            Subgroup { id: 2, profit: 30.0, node_ids: vec![3], ..Default::default() },
        ],
        clusters: vec![
            Cluster { id: 0, subgroup_ids: vec![0] },
            Cluster { id: 1, subgroup_ids: vec![1] },
            Cluster { id: 2, subgroup_ids: vec![2] },
        ],
        vehicles: vec![
            Vehicle { id: 0, budget: 100.0, start_node_id: 0, end_node_id: 0 },
        ],
        ..Default::default()
    };

    let data = StopData::new(&instance);
    let vars = initialize_variables(&mut model, &instance, &data)?;

    let comp = vec![1, 2];
    let num_nodes = instance.nodes.len();

    // 1. Validate DFJ expression: sum_{u, v in comp, u != v} x_{r, u, v}
    let dfj_expr = build_dfj_expr(&vars, 0, &comp);
    let constr_dfj = model.add_constr("dfj_cut", c!(dfj_expr <= (comp.len() - 1) as f64))?;

    // 2. Validate Cut-Set expression: sum_{u not in comp, v in comp} x_{r, u, v} >= y_{r, v}
    let cut_set_expr = build_cut_set_expr(&vars, 0, &comp, num_nodes);
    let constr_cut_1 = model.add_constr("cutset_node_1", c!(cut_set_expr.clone() >= vars.y[0][1]))?;
    let constr_cut_2 = model.add_constr("cutset_node_2", c!(cut_set_expr >= vars.y[0][2]))?;

    model.update()?;

    // Verify constraints were properly added to the model
    assert!(model.get_attr(attr::NumConstrs)? >= 3);
    assert!(matches!(model.get_obj_attr(attr::Sense, &constr_dfj)?, grb::ConstrSense::Less));
    assert!(matches!(model.get_obj_attr(attr::Sense, &constr_cut_1)?, grb::ConstrSense::Greater));
    assert!(matches!(model.get_obj_attr(attr::Sense, &constr_cut_2)?, grb::ConstrSense::Greater));

    // Verify StopSubtourCallback creation caches both x and y variables
    let cb = StopSubtourCallback::new(&vars, instance.vehicles.len(), num_nodes, data.depot_id);
    assert_eq!(cb.cache.x_vars[0].len(), num_nodes * (num_nodes - 1));
    assert_eq!(cb.cache.y_vars[0].len(), num_nodes);

    Ok(())
}

#[test]
fn test_stop_solver_eliminates_isolated_subtour() {
    // Instance with:
    // - Depot 0 at (0, 0)
    // - Cluster 1 at (0, 10): roundtrip cost = 20.0, profit = 50.0
    // - Cluster 2 at (10, 0): roundtrip cost = 20.0, profit = 40.0
    // - Cluster 3 (isolated nodes 3 and 4) at (100, 100) and (100, 101):
    //   Distance between 3 and 4 is 1.0 (subtour cost 2.0).
    //   Distance to depot is ~141.4 (> budget of 30.0).
    //   Profit is huge (500.0).
    // If subtours were not eliminated, solver would pick isolated cycle {3, 4}.
    // With DFJ + Cut-Set, cycle {3, 4} is strictly eliminated and solver visits {1} and {2}.
    let instance = Instance {
        name: "test_isolated_subtour_elimination".to_string(),
        metric: Metric::Euc2d,
        nodes: vec![
            Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, parent_subgroup_ids: HashSet::new() },
            Node { id: 1, point: Point3 { x: 0.0, y: 10.0, z: 0.0 }, parent_subgroup_ids: HashSet::from([1]) },
            Node { id: 2, point: Point3 { x: 10.0, y: 0.0, z: 0.0 }, parent_subgroup_ids: HashSet::from([2]) },
            Node { id: 3, point: Point3 { x: 100.0, y: 100.0, z: 0.0 }, parent_subgroup_ids: HashSet::from([3]) },
            Node { id: 4, point: Point3 { x: 100.0, y: 101.0, z: 0.0 }, parent_subgroup_ids: HashSet::from([3]) },
        ],
        subgroups: vec![
            Subgroup { id: 0, profit: 0.0, node_ids: vec![0], parent_cluster_ids: HashSet::from([0]) },
            Subgroup { id: 1, profit: 50.0, node_ids: vec![1], parent_cluster_ids: HashSet::from([1]) },
            Subgroup { id: 2, profit: 40.0, node_ids: vec![2], parent_cluster_ids: HashSet::from([2]) },
            Subgroup { id: 3, profit: 500.0, node_ids: vec![3, 4], parent_cluster_ids: HashSet::from([3]) },
        ],
        clusters: vec![
            Cluster { id: 0, subgroup_ids: vec![0] },
            Cluster { id: 1, subgroup_ids: vec![1] },
            Cluster { id: 2, subgroup_ids: vec![2] },
            Cluster { id: 3, subgroup_ids: vec![3] },
        ],
        vehicles: vec![
            Vehicle { id: 0, budget: 30.0, start_node_id: 0, end_node_id: 0 },
            Vehicle { id: 1, budget: 30.0, start_node_id: 0, end_node_id: 0 },
        ],
        ..Default::default()
    };

    let options = StopOptions {
        mode: SolverMode::Exact,
        time_limit: Some(15),
        max_iterations: 10,
        gurobi_params_file: None,
    };

    let sol = stop::solve(&instance, &options).expect("STOP exact solve failed");

    // 1. Must achieve optimal solution without hanging
    assert_eq!(sol.status, SolutionStatus::Optimal, "STOP must converge to Optimal");

    // 2. Optimal score must be 90.0 (clusters 1 and 2: 50.0 + 40.0), NOT 500+ from isolated cycle
    assert_eq!(sol.total_score, 90.0, "Optimal score must be 90.0 without isolated subtours");

    // 3. Verify no isolated subtours exist: every route must start and end at depot 0
    assert_eq!(sol.routes.len(), 2, "Both vehicles should have routes");
    for route in &sol.routes {
        assert_eq!(route.path.first(), Some(&0), "Route must start at depot 0");
        assert_eq!(route.path.last(), Some(&0), "Route must end at depot 0");

        // The isolated nodes 3 and 4 must never be visited
        assert!(
            !route.path.contains(&3),
            "Route must not contain isolated node 3"
        );
        assert!(
            !route.path.contains(&4),
            "Route must not contain isolated node 4"
        );

        // Cost must be <= budget (30.0)
        assert!(
            route.cost <= 30.0 + 1e-4,
            "Route cost ({}) exceeds vehicle budget (30.0)",
            route.cost
        );
    }
}
