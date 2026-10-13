#![allow(clippy::useless_conversion)]

use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::exact::variable::DecisionVariables;

pub fn apply_preprocessing(
    model: &mut Model,
    variables: &DecisionVariables,
    instance: &Instance,
    data: &StopData,
) -> grb::Result<()> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();
    let depot = data.depot_id;

    let mut node_incompatible_all = vec![true; num_nodes];
    node_incompatible_all[depot] = false;

    for r in 0..num_vehicles {
        let budget_r = instance.vehicles[r].budget;
        let mut node_incompatible_r = vec![false; num_nodes];

        // 1. Incompatible nodes for vehicle r: c(0, i) + c(i, 0) > budget_r
        for i in 0..num_nodes {
            if i == depot {
                continue;
            }

            let round_trip = instance.get_distance(depot, i) + instance.get_distance(i, depot);
            if round_trip > budget_r {
                node_incompatible_r[i] = true;
                for j in 0..num_nodes {
                    if i != j {
                        model.add_constr(&format!("inacc_arc_in_v{}_{}_{}", r, j, i), c!(variables.x[r][j][i] == 0.0))?;
                        model.add_constr(&format!("inacc_arc_out_v{}_{}_{}", r, i, j), c!(variables.x[r][i][j] == 0.0))?;
                    }
                }
            } else {
                node_incompatible_all[i] = false;
            }
        }

        // 2. Incompatible arcs for vehicle r: c(0, i) + c(i, j) + c(j, 0) > budget_r
        for i in 0..num_nodes {
            if i == depot || node_incompatible_r[i] {
                continue;
            }
            let c_0_i = instance.get_distance(depot, i);

            for (j, &j_incompatible) in node_incompatible_r.iter().enumerate() {
                if j == depot || j == i || j_incompatible {
                    continue;
                }
                let c_i_j = instance.get_distance(i, j);
                let c_j_0 = instance.get_distance(j, depot);

                if c_0_i + c_i_j + c_j_0 > budget_r {
                    model.add_constr(
                        &format!("incomp_arc_v{}_{}_{}", r, i, j),
                        c!(variables.x[r][i][j] == 0.0),
                    )?;
                }
            }
        }
    }

    // 3. Incompatible clusters: all nodes in cluster are incompatible across all vehicles
    for (k, cluster) in data.clusters.iter().enumerate() {
        let all_incompatible = cluster.node_ids.iter().all(|&nid| node_incompatible_all[nid]);
        if all_incompatible {
            model.add_constr(&format!("inacc_cl_{}", k), c!(variables.y[k] == 0.0))?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::stop::exact::variable::initialize_variables;

    #[test]
    fn test_stop_preprocess() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_stop_prep", &env)?;
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
        let data = StopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        apply_preprocessing(&mut model, &vars, &instance, &data)?;
        model.update()?;

        assert!(model.get_attr(attr::NumConstrs)? > 0);
        Ok(())
    }
}
