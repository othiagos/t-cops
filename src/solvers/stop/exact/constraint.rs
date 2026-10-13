#![allow(clippy::useless_conversion)]

use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::exact::variable::DecisionVariables;

/// Flow conservation at every node j \in V, for each vehicle r and vehicle r leaves depot at most once
pub fn flow_conservation(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &StopData,
    num_nodes: usize,
) -> grb::Result<()> {
    let num_vehicles = variables.x.len();
    let depot = data.depot_id;

    for r in 0..num_vehicles {
        for j in 0..num_nodes {
            let sum_in: grb::Expr = (0..num_nodes)
                .filter(|&i| i != j)
                .map(|i| 1.0 * variables.x[r][i][j])
                .sum();

            let sum_out: grb::Expr = (0..num_nodes)
                .filter(|&i| i != j)
                .map(|i| 1.0 * variables.x[r][j][i])
                .sum();

            model.add_constr(&format!("flow_cons_v{}_n{}", r, j), c!(sum_in == sum_out))?;
        }

        let depot_out: grb::Expr = (0..num_nodes)
            .filter(|&j| j != depot)
            .map(|j| 1.0 * variables.x[r][depot][j])
            .sum();
        model.add_constr(&format!("depot_out_v{}", r), c!(depot_out <= 1.0))?;
    }

    Ok(())
}

/// Cluster entry
pub fn cluster_entry(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &StopData,
    num_nodes: usize,
) -> grb::Result<()> {
    let num_vehicles = variables.x.len();

    for (k, cluster) in data.clusters.iter().enumerate() {
        let entry_expr: grb::Expr = cluster
            .node_ids
            .iter()
            .flat_map(|&j| {
                (0..num_vehicles).flat_map(move |r| {
                    (0..num_nodes)
                        .filter(|i| !cluster.node_ids.contains(i))
                        .map(move |i| 1.0 * variables.x[r][i][j])
                })
            })
            .sum();

        let y_k = variables.y[k];
        model.add_constr(&format!("cluster_entry_{}", k), c!(entry_expr == y_k))?;
    }

    Ok(())
}

/// Prohibit intra-cluster arcs
pub fn no_intra_cluster_arcs(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &StopData,
) -> grb::Result<()> {
    let num_vehicles = variables.x.len();

    for (k, cluster) in data.clusters.iter().enumerate() {
        for &i in &cluster.node_ids {
            for &j in &cluster.node_ids {
                if i != j {
                    for r in 0..num_vehicles {
                        model.add_constr(
                            &format!("no_intra_v{}_cl{}_{}_{}", r, k, i, j),
                            c!(variables.x[r][i][j] == 0.0),
                        )?;
                    }
                }
            }
        }
    }

    Ok(())
}

/// Vehicle budget limit
pub fn vehicle_budget(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for r in 0..num_vehicles {
        let budget = instance.vehicles[r].budget;

        let total_cost_expr: grb::Expr = (0..num_nodes)
            .flat_map(|i| {
                (0..num_nodes)
                    .filter(move |&j| i != j)
                    .map(move |j| instance.get_distance(i, j) * variables.x[r][i][j])
            })
            .sum();

        model.add_constr(&format!("budget_v{}", r), c!(total_cost_expr <= budget))?;
    }

    Ok(())
}

/// Each customer visited at most once across all vehicles
pub fn unique_customer_visit(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &StopData,
    num_nodes: usize,
) -> grb::Result<()> {
    let num_vehicles = variables.x.len();
    let depot = data.depot_id;

    for j in 0..num_nodes {
        if j == depot {
            continue;
        }

        let visit_expr: grb::Expr = (0..num_vehicles)
            .flat_map(|r| {
                (0..num_nodes)
                    .filter(move |&i| i != j)
                    .map(move |i| 1.0 * variables.x[r][i][j])
            })
            .sum();

        model.add_constr(&format!("unique_visit_{}", j), c!(visit_expr <= 1.0))?;
    }

    Ok(())
}

/// Symmetry breaking cuts for vehicles with identical budgets
pub fn symmetry_breaking(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &StopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for r in 0..(num_vehicles.saturating_sub(1)) {
        if (instance.vehicles[r].budget - instance.vehicles[r + 1].budget).abs() < 1e-6 {
            let profit_r: grb::Expr = (0..num_nodes)
                .filter(|&j| j != data.depot_id && data.node_profits[j] > 0.0)
                .flat_map(|j| {
                    let p = data.node_profits[j];
                    (0..num_nodes)
                        .filter(move |&i| i != j)
                        .map(move |i| p * variables.x[r][i][j])
                })
                .sum();

            let profit_r_next: grb::Expr = (0..num_nodes)
                .filter(|&j| j != data.depot_id && data.node_profits[j] > 0.0)
                .flat_map(|j| {
                    let p = data.node_profits[j];
                    (0..num_nodes)
                        .filter(move |&i| i != j)
                        .map(move |i| p * variables.x[r + 1][i][j])
                })
                .sum();

            model.add_constr(&format!("symm_v{}_v{}", r, r + 1), c!(profit_r >= profit_r_next))?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::stop::exact::variable::initialize_variables;

    fn create_test_instance() -> Instance {
        Instance {
            name: "test_stop_constr".to_string(),
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 3.0, y: 4.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 6.0, y: 8.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1], ..Default::default() },
                Subgroup { id: 2, profit: 20.0, node_ids: vec![2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1, 2] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 100.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_add_all_stop_constraints() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_stop_constr", &env)?;
        let instance = create_test_instance();
        let data = StopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        let num_nodes = instance.nodes.len();
        flow_conservation(&mut model, &vars, &data, num_nodes)?;
        cluster_entry(&mut model, &vars, &data, num_nodes)?;
        no_intra_cluster_arcs(&mut model, &vars, &data)?;
        vehicle_budget(&mut model, &vars, &instance)?;
        unique_customer_visit(&mut model, &vars, &data, num_nodes)?;
        symmetry_breaking(&mut model, &vars, &instance, &data)?;

        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }
}
