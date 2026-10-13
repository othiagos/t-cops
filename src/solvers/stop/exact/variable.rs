use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;

pub struct DecisionVariables {
    pub x: Vec<Vec<Vec<Var>>>,
    pub y: Vec<Var>,
}

pub fn initialize_variables(
    model: &mut Model,
    instance: &Instance,
    data: &StopData,
) -> grb::Result<DecisionVariables> {
    let num_nodes = instance.nodes.len();
    let num_clusters = data.clusters.len();
    let num_vehicles = instance.vehicles.len();

    // 1. x[r][i][j]: binary arc variables per vehicle
    let mut x: Vec<Vec<Vec<Var>>> = Vec::with_capacity(num_vehicles);
    for r in 0..num_vehicles {
        let mut x_r: Vec<Vec<Var>> = Vec::with_capacity(num_nodes);
        for i in 0..num_nodes {
            let mut x_ri = Vec::with_capacity(num_nodes);
            for j in 0..num_nodes {
                if i != j {
                    let var = add_binvar!(model, name: &format!("x_{}_{}_{}", r, i, j))?;
                    x_ri.push(var);
                } else {
                    let diag = add_var!(
                        model,
                        Binary,
                        name: &format!("x_diag_{}_{}", r, i),
                        bounds: 0.0_f32..0.0_f32
                    )?;
                    x_ri.push(diag);
                }
            }
            x_r.push(x_ri);
        }
        x.push(x_r);
    }

    // 2. y[k]: binary cluster visited variables
    let mut y = Vec::with_capacity(num_clusters);
    for k in 0..num_clusters {
        let var = add_binvar!(model, name: &format!("y_cl_{}", k))?;
        y.push(var);
    }

    Ok(DecisionVariables { x, y })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Subgroup, Vehicle};

    #[test]
    fn test_stop_variable_init() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_stop_var", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, ..Default::default() },
                Node { id: 1, ..Default::default() },
                Node { id: 2, ..Default::default() },
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
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };

        let data = StopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        assert_eq!(vars.x.len(), 1);
        assert_eq!(vars.x[0].len(), 3);
        assert_eq!(vars.x[0][0].len(), 3);
        assert_eq!(vars.y.len(), 1);

        Ok(())
    }
}
