#![allow(clippy::useless_conversion)]

use grb::prelude::*;
use std::collections::HashSet;

use crate::common::instance::Instance;
use crate::solvers::exact::gurobi::ilp::DecisionVariables;

pub fn intermediate_flow_conservation(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    for k in 0..instance.vehicles.len() {
        let start_node = instance.vehicles[k].start_node_id;
        let end_node = instance.vehicles[k].end_node_id;

        for i in 0..num_nodes {
            if i == start_node || i == end_node {
                continue;
            }

            let sum_in: grb::Expr = (0..num_nodes)
                .filter(|&j| i != j)
                .map(|j| 1.0 * variable.x[k][j][i])
                .sum();

            let sum_out: grb::Expr = (0..num_nodes)
                .filter(|&j| i != j)
                .map(|j| 1.0 * variable.x[k][i][j])
                .sum();

            let y_var = variable.y[k][i];

            model.add_constr(&format!("flow_in_y_v{}_n{}", k, i), c!(sum_in == y_var))?;
            model.add_constr(&format!("flow_out_y_v{}_n{}", k, i), c!(sum_out == y_var))?;
        }
    }
    Ok(())
}

pub fn unique_visit(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    let mut depot_nodes = HashSet::new();
    for vehicle in instance.vehicles.iter() {
        depot_nodes.insert(vehicle.start_node_id);
        depot_nodes.insert(vehicle.end_node_id);
    }

    for i in 0..num_nodes {
        if depot_nodes.contains(&i) {
            continue;
        }

        let total_visits: grb::Expr = (0..instance.vehicles.len())
            .map(|k| 1.0 * variable.y[k][i])
            .sum();

        model.add_constr(&format!("unique_visit_n{}", i), c!(total_visits <= 1.0_f32))?;
    }

    Ok(())
}

pub fn depot_flow(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    for k in 0..instance.vehicles.len() {
        let start_node = instance.vehicles[k].start_node_id;
        let end_node = instance.vehicles[k].end_node_id;

        if start_node != end_node {
            continue;
        }

        let sum_out_start: grb::Expr = (0..num_nodes)
            .filter(|&j| start_node != j)
            .map(|j| 1.0 * variable.x[k][start_node][j])
            .sum();

        let sum_in_end: grb::Expr = (0..num_nodes)
            .filter(|&j| end_node != j)
            .map(|j| 1.0 * variable.x[k][j][end_node])
            .sum();

        let out_clone = sum_out_start.clone();
        let in_clone = sum_in_end.clone();

        model.add_constr(
            &format!("depot_flow_out_start_v{}", k),
            c!(out_clone <= 1.0),
        )?;
        model.add_constr(&format!("depot_flow_in_end_v{}", k), c!(in_clone <= 1.0))?;
        model.add_constr(
            &format!("depot_flow_equal_v{}", k),
            c!(sum_out_start == sum_in_end),
        )?;
    }

    Ok(())
}

pub fn depot_blocks(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    for k in 0..instance.vehicles.len() {
        let start_node = instance.vehicles[k].start_node_id;
        let end_node = instance.vehicles[k].end_node_id;

        if start_node == end_node {
            continue;
        }

        let sum_in_start: grb::Expr = (0..num_nodes)
            .filter(|&j| start_node != j)
            .map(|j| 1.0 * variable.x[k][j][start_node])
            .sum();

        let sum_out_end: grb::Expr = (0..num_nodes)
            .filter(|&j| end_node != j)
            .map(|j| 1.0 * variable.x[k][end_node][j])
            .sum();

        model.add_constr(
            &format!("depot_block_in_start_v{}", k),
            c!(sum_in_start == 0.0),
        )?;
        model.add_constr(
            &format!("depot_block_out_end_v{}", k),
            c!(sum_out_end == 0.0),
        )?;
    }

    Ok(())
}

pub fn logical_visit(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    for k in 0..instance.vehicles.len() {
        let start_node = instance.vehicles[k].start_node_id;
        let end_node = instance.vehicles[k].end_node_id;

        let vehicle_used: grb::Expr = (0..num_nodes)
            .filter(|&j| j != start_node)
            .map(|j| variable.x[k][start_node][j])
            .sum();

        for i in 0..num_nodes {
            if i == start_node || i == end_node {
                continue;
            }

            model.add_constr(
                &format!("logical_visit_v{}_n{}", k, i),
                c!(variable.y[k][i] <= vehicle_used.clone()),
            )?;
        }
    }

    Ok(())
}


pub fn logical_physical_z_leq_y(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    for (i, node) in instance.nodes.iter().enumerate() {
        if node.parent_subgroup_ids.is_empty() {
            continue;
        }

        let sum_y_physic: grb::Expr = (0..instance.vehicles.len())
            .map(|k| 1.0 * variable.y[k][i])
            .sum();

        for &s_id in &node.parent_subgroup_ids {
            model.add_constr(
                &format!("logic_physic_z_leq_y_n{}_s{}", i, s_id),
                c!(variable.z[s_id] <= sum_y_physic.clone()),
            )?;
        }
    }

    Ok(())
}

pub fn logical_physical_y_leq_z(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    for (i, node) in instance.nodes.iter().enumerate() {
        if node.parent_subgroup_ids.is_empty() {
            continue;
        }

        let sum_y_physic: grb::Expr = (0..instance.vehicles.len())
            .map(|k| 1.0 * variable.y[k][i])
            .sum();

        let sum_z_logic: grb::Expr = node
            .parent_subgroup_ids
            .iter()
            .map(|&s_id| 1.0 * variable.z[s_id])
            .sum();

        model.add_constr(
            &format!("logic_physic_y_leq_sum_z_n{}", i),
            c!(sum_y_physic <= sum_z_logic),
        )?;
    }

    Ok(())
}

pub fn cluster(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    for (c_id, cluster) in instance.clusters.iter().enumerate() {
        let sum_z_subgroups: grb::Expr = cluster
            .subgroup_ids
            .iter()
            .map(|&subgroup_id| 1.0 * variable.z[subgroup_id])
            .sum();

        model.add_constr(
            &format!("cluster_c{}", c_id),
            c!(sum_z_subgroups == variable.w[c_id]),
        )?;
    }

    Ok(())
}

pub fn budget(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_nodes = instance.nodes.len();

    for k in 0..instance.vehicles.len() {
        let vehicle_budget = instance.vehicles[k].budget;

        let total_cost_expr: grb::Expr = (0..num_nodes)
            .flat_map(|i| {
                (0..num_nodes)
                    .filter(move |&j| i != j)
                    .map(move |j| instance.get_distance(i, j) * variable.x[k][i][j])
            })
            .sum();

        model.add_constr(
            &format!("budget_v{}", k),
            c!(total_cost_expr <= vehicle_budget),
        )?;
    }

    Ok(())
}

pub fn symmetry_breaking(
    model: &mut Model,
    variable: &DecisionVariables,
    instance: &Instance,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    if num_vehicles <= 1 {
        return Ok(());
    }

    let num_nodes = instance.nodes.len();

    for r in 0..(num_vehicles.saturating_sub(1)) {
        let v_curr = &instance.vehicles[r];
        let v_next = &instance.vehicles[r + 1];

        let same_budget = (v_curr.budget - v_next.budget).abs() < 1e-6;
        let same_start = v_curr.start_node_id == v_next.start_node_id;
        let same_end = v_curr.end_node_id == v_next.end_node_id;

        if same_budget && same_start && same_end {
            let start = v_curr.start_node_id;
            let end = v_curr.end_node_id;

            for i in 0..num_nodes {
                if i == start || i == end {
                    continue;
                }

                let sum_prev: grb::Expr = (0..i)
                    .filter(|&j| j != start && j != end)
                    .map(|j| 1.0 * variable.y[r][j])
                    .sum();

                model.add_constr(
                    &format!("symm_order_v{}_v{}_n{}", r, r + 1, i),
                    c!(variable.y[r + 1][i] <= sum_prev),
                )?;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Subgroup, Vehicle};
    use crate::solvers::exact::gurobi::variable;
    use std::collections::HashSet;

    fn create_test_instance() -> Instance {
        Instance {
            nodes: vec![
                Node {
                    id: 0,
                    parent_subgroup_ids: HashSet::from([0]),
                    ..Default::default()
                },
                Node {
                    id: 1,
                    parent_subgroup_ids: HashSet::from([0]),
                    ..Default::default()
                },
            ],
            subgroups: vec![Subgroup {
                id: 0,
                profit: 10.0,
                node_ids: vec![0, 1],
                parent_cluster_ids: HashSet::from([0]),
            }],
            clusters: vec![Cluster {
                id: 0,
                subgroup_ids: vec![0],
            }],
            vehicles: vec![Vehicle {
                id: 0,
                budget: 50.0,
                start_node_id: 0,
                end_node_id: 0,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_intermediate_flow_conservation_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("inter_flow_test", &env)?;
        let instance = create_test_instance();

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        intermediate_flow_conservation(&mut model, &vars, &instance)?;
        model.update()?;
        assert_eq!(model.get_attr(attr::NumConstrs)?, 2);

        Ok(())
    }

    #[test]
    fn test_depot_flow_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("depot_flow_test", &env)?;
        let instance = create_test_instance();

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        depot_flow(&mut model, &vars, &instance)?;
        model.update()?;
        assert_eq!(model.get_attr(attr::NumConstrs)?, 3);

        Ok(())
    }

    #[test]
    fn test_depot_blocks_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("depot_blocks_test", &env)?;
        let mut instance = create_test_instance();
        instance.vehicles[0].start_node_id = 0;
        instance.vehicles[0].end_node_id = 1;

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        depot_blocks(&mut model, &vars, &instance)?;
        model.update()?;
        assert_eq!(model.get_attr(attr::NumConstrs)?, 2);

        Ok(())
    }

    #[test]
    fn test_logical_physical_z_leq_y_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("log_phys_z_y_test", &env)?;
        let instance = create_test_instance();

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        logical_physical_z_leq_y(&mut model, &vars, &instance)?;
        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }

    #[test]
    fn test_logical_physical_y_leq_z_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("log_phys_y_z_test", &env)?;
        let instance = create_test_instance();

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        logical_physical_y_leq_z(&mut model, &vars, &instance)?;
        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }

    #[test]
    fn test_all_constraints_building() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("constr_test", &env)?;
        let instance = create_test_instance();

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;

        let vars = DecisionVariables { x, y, z, w };

        intermediate_flow_conservation(&mut model, &vars, &instance)?;
        depot_flow(&mut model, &vars, &instance)?;
        depot_blocks(&mut model, &vars, &instance)?;
        logical_visit(&mut model, &vars, &instance)?;
        logical_physical_z_leq_y(&mut model, &vars, &instance)?;
        logical_physical_y_leq_z(&mut model, &vars, &instance)?;
        cluster(&mut model, &vars, &instance)?;
        budget(&mut model, &vars, &instance)?;
        symmetry_breaking(&mut model, &vars, &instance)?;

        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }

    #[test]
    fn test_symmetry_breaking_homogeneous_vehicles() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("symm_homo_test", &env)?;
        let mut instance = create_test_instance();
        instance.vehicles.push(Vehicle {
            id: 1,
            budget: 50.0,
            start_node_id: 0,
            end_node_id: 0,
        });

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        symmetry_breaking(&mut model, &vars, &instance)?;
        model.update()?;
        assert!(model.get_attr(attr::NumConstrs)? > 0);

        Ok(())
    }

    #[test]
    fn test_symmetry_breaking_heterogeneous_vehicles() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("symm_hetero_test", &env)?;
        let mut instance = create_test_instance();
        instance.vehicles.push(Vehicle {
            id: 1,
            budget: 80.0, // Different budget
            start_node_id: 0,
            end_node_id: 0,
        });

        let x = variable::initialize_x(&mut model, &instance)?;
        let y = variable::initialize_y(&mut model, &instance)?;
        let z = variable::initialize_z(&mut model, &instance)?;
        let w = variable::initialize_w(&mut model, &instance)?;
        let vars = DecisionVariables { x, y, z, w };

        symmetry_breaking(&mut model, &vars, &instance)?;
        model.update()?;
        assert_eq!(model.get_attr(attr::NumConstrs)?, 0);

        Ok(())
    }
}
