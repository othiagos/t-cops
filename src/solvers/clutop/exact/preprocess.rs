#![allow(clippy::useless_conversion)]

use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::exact::variable::DecisionVariables;

pub fn apply_preprocessing(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();
    let depot = data.depot_id;

    // Max budget across all vehicles
    let max_fleet_budget = instance
        .vehicles
        .iter()
        .map(|v| v.budget)
        .fold(0.0_f64, f64::max);

    let mut inaccessible_customers = vec![false; num_nodes];

    // Check inaccessible customers: 2 * c(0, i) > budget_r
    for (i, inaccessible) in inaccessible_customers.iter_mut().enumerate() {
        if i == depot {
            continue;
        }

        let dist_to_depot = instance.get_distance(depot, i);
        let round_trip = 2.0 * dist_to_depot;

        for r in 0..num_vehicles {
            if round_trip > instance.vehicles[r].budget {
                model.add_constr(
                    &format!("inacc_cust_v{}_n{}", r, i),
                    c!(variables.y[r][i] == 0.0),
                )?;
            }
        }

        if round_trip > max_fleet_budget {
            *inaccessible = true;
        }
    }

    // Check inaccessible clusters: if any customer is inaccessible to the whole fleet
    for (c_idx, cluster) in data.clusters.iter().enumerate() {
        let has_inacc_customer = cluster
            .node_ids
            .iter()
            .any(|&nid| inaccessible_customers[nid]);

        if has_inacc_customer {
            model.add_constr(
                &format!("inacc_cluster_c{}", c_idx),
                c!(variables.z[c_idx] == 0.0),
            )?;
        }
    }

    // Incompatibility cuts between pairs of customers 
    for r in 0..num_vehicles {
        let budget_r = instance.vehicles[r].budget;

        for i in 0..num_nodes {
            if i == depot || inaccessible_customers[i] {
                continue;
            }
            let dist_0_i = instance.get_distance(depot, i);

            for (j, &j_inaccessible) in inaccessible_customers.iter().enumerate().skip(i + 1) {
                if j == depot || j_inaccessible {
                    continue;
                }
                let dist_0_j = instance.get_distance(depot, j);
                let dist_i_j = instance.get_distance(i, j);

                let min_tour_cost = dist_0_i + dist_i_j + dist_0_j;
                if min_tour_cost > budget_r {
                    model.add_constr(
                        &format!("incomp_v{}_n{}_n{}", r, i, j),
                        c!(variables.y[r][i] + variables.y[r][j] <= 1.0),
                    )?;
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::clutop::exact::variable::initialize_variables;

    #[test]
    fn test_preprocessing_inaccessible() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_prep", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 100.0, y: 0.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 10.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };
        let data = CluTopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        apply_preprocessing(&mut model, &vars, &instance, &data)?;
        model.update()?;

        assert!(model.get_attr(attr::NumConstrs)? > 0);
        Ok(())
    }
}
