#![allow(clippy::useless_conversion)]

use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::exact::variable::DecisionVariables;

/// (2) Each customer visited by at most one vehicle
pub fn unique_customer_visit(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for i in 0..num_nodes {
        if i == data.depot_id {
            continue;
        }

        let sum_y: grb::Expr = (0..num_vehicles).map(|r| 1.0 * variables.y[r][i]).sum();
        model.add_constr(&format!("unique_visit_{}", i), c!(sum_y <= 1.0))?;
    }

    Ok(())
}

/// (3) Flow/degree conservation
pub fn degree_conservation(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for r in 0..num_vehicles {
        for i in 0..num_nodes {
            let sum_edges: grb::Expr = (0..num_nodes)
                .filter(|&j| j != i)
                .map(|j| 1.0 * variables.x[r][i][j])
                .sum();

            let target = 2.0 * variables.y[r][i];
            model.add_constr(&format!("deg_v{}_n{}", r, i), c!(sum_edges == target))?;
        }
    }

    Ok(())
}

/// (4) Budget limit per vehicle
pub fn budget(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for r in 0..num_vehicles {
        let max_time = instance.vehicles[r].budget;

        let total_cost_expr: grb::Expr = (0..num_nodes)
            .flat_map(|i| {
                ((i + 1)..num_nodes).map(move |j| {
                    instance.get_distance(i, j) * variables.x[r][i][j]
                })
            })
            .sum();

        model.add_constr(&format!("budget_v{}", r), c!(total_cost_expr <= max_time))?;
    }

    Ok(())
}

/// (6) Cluster profit (ALL-or-nothing)
pub fn cluster_profit(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();

    for (c_idx, cluster) in data.clusters.iter().enumerate() {
        let z_k = variables.z[c_idx];

        for &node_id in &cluster.node_ids {
            let sum_y: grb::Expr = (0..num_vehicles)
                .map(|r| 1.0 * variables.y[r][node_id])
                .sum();

            model.add_constr(
                &format!("cluster_all_c{}_n{}", c_idx, node_id),
                c!(z_k <= sum_y),
            )?;
        }
    }

    Ok(())
}

/// (10) Valid inequality
pub fn cluster_coverage_cut(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();

    for i in 0..num_nodes {
        if i == data.depot_id {
            continue;
        }

        let clusters = &data.customer_clusters[i];
        if clusters.is_empty() {
            // Node does not belong to any cluster, so it should never be visited
            for r in 0..num_vehicles {
                model.add_constr(&format!("no_cluster_v{}_n{}", r, i), c!(variables.y[r][i] == 0.0))?;
            }
            continue;
        }

        let sum_y: grb::Expr = (0..num_vehicles).map(|r| 1.0 * variables.y[r][i]).sum();
        let sum_z: grb::Expr = clusters.iter().map(|&c_idx| 1.0 * variables.z[c_idx]).sum();

        model.add_constr(&format!("coverage_cut_n{}", i), c!(sum_y <= sum_z))?;
    }

    Ok(())
}

/// (13) Symmetry breaking cuts
pub fn symmetry_breaking(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    if num_vehicles <= 1 {
        return Ok(());
    }

    // Symmetry breaking cuts are only valid for homogeneous vehicles (identical budgets).
    // For heterogeneous vehicles, ordering profits lexicographically cuts off the optimal solution.
    let all_budgets_equal = instance
        .vehicles
        .windows(2)
        .all(|w| (w[0].budget - w[1].budget).abs() < 1e-6);
    if !all_budgets_equal {
        return Ok(());
    }

    let num_nodes = instance.nodes.len();
    let mut p_scores = vec![0.0; num_nodes];

    for cluster in &data.clusters {
        let size = cluster.node_ids.len() as f64;
        if size > 0.0 {
            let contrib = cluster.profit / size;
            for &nid in &cluster.node_ids {
                if nid < num_nodes {
                    p_scores[nid] += contrib;
                }
            }
        }
    }

    for r in 0..(num_vehicles - 1) {
        let sum_prev: grb::Expr = (0..num_nodes)
            .filter(|&i| i != data.depot_id && p_scores[i] > 0.0)
            .map(|i| p_scores[i] * variables.y[r][i])
            .sum();

        let sum_next: grb::Expr = (0..num_nodes)
            .filter(|&i| i != data.depot_id && p_scores[i] > 0.0)
            .map(|i| p_scores[i] * variables.y[r + 1][i])
            .sum();

        model.add_constr(&format!("symm_break_{}", r), c!(sum_next <= sum_prev))?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::clutop::exact::variable::initialize_variables;

    fn create_test_instance() -> Instance {
        Instance {
            name: "constr_test".to_string(),
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 3.0, y: 4.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 6.0, y: 8.0, z: 0.0 }, ..Default::default() },
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
                Vehicle { id: 0, budget: 100.0, start_node_id: 0, end_node_id: 0 },
                Vehicle { id: 1, budget: 100.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_add_all_constraints() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_constr", &env)?;
        let instance = create_test_instance();
        let data = CluTopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        unique_customer_visit(&mut model, &vars, &instance, &data)?;
        degree_conservation(&mut model, &vars, &instance)?;
        budget(&mut model, &vars, &instance)?;
        cluster_profit(&mut model, &vars, &instance, &data)?;
        cluster_coverage_cut(&mut model, &vars, &instance, &data)?;
        symmetry_breaking(&mut model, &vars, &instance, &data)?;

        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }

    #[test]
    fn test_symmetry_breaking_heterogeneous_vehicles() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_symm_hetero", &env)?;
        let mut instance = create_test_instance();
        instance.vehicles[0].budget = 50.0;
        instance.vehicles[1].budget = 100.0;
        let data = CluTopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        symmetry_breaking(&mut model, &vars, &instance, &data)?;
        model.update()?;
        assert_eq!(model.get_attr(attr::NumConstrs)?, 0);

        Ok(())
    }
}
