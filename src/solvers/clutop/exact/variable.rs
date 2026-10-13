use grb::prelude::*;
use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;

pub struct DecisionVariables {
    pub x: Vec<Vec<Vec<Var>>>,
    pub y: Vec<Vec<Var>>,
    pub z: Vec<Var>,
}

pub fn initialize_variables(
    model: &mut Model,
    instance: &Instance,
    data: &CluTopData,
) -> grb::Result<DecisionVariables> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();
    let num_clusters = data.clusters.len();

    // Initialize x[k][i][j] for undirected edges
    let mut x: Vec<Vec<Vec<Var>>> = Vec::with_capacity(num_vehicles);
    for k in 0..num_vehicles {
        let mut x_k: Vec<Vec<Option<Var>>> = vec![vec![None; num_nodes]; num_nodes];

        let mut reverse_edges = Vec::new();
        for (i, row) in x_k.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate().skip(i + 1) {
                let var = add_binvar!(model, name: &format!("x_{}_{}_{}", k, i, j))?;
                *cell = Some(var);
                reverse_edges.push((j, i, var));
            }

            let diag_var = add_var!(
                model,
                Binary,
                name: &format!("x_diag_{}_{}", k, i),
                bounds: 0.0_f32..0.0_f32
            )?;

            row[i] = Some(diag_var);
        }

        for (i, j, var) in reverse_edges {
            x_k[i][j] = Some(var);
        }

        let x_k_final: Vec<Vec<Var>> = x_k
            .into_iter()
            .map(|row| row.into_iter().map(|opt| opt.unwrap()).collect())
            .collect();

        x.push(x_k_final);
    }

    // Initialize y[k][i] for each vehicle and node (including depot)
    let mut y: Vec<Vec<Var>> = Vec::with_capacity(num_vehicles);
    for k in 0..num_vehicles {
        let mut y_k: Vec<Var> = Vec::with_capacity(num_nodes);
        for i in 0..num_nodes {
            let var = add_binvar!(model, name: &format!("y_{}_{}", k, i))?;
            y_k.push(var);
        }
        y.push(y_k);
    }

    // Initialize z[c] for each customer cluster
    let mut z: Vec<Var> = Vec::with_capacity(num_clusters);
    for c in 0..num_clusters {
        let var = add_binvar!(model, name: &format!("z_{}", c))?;
        z.push(var);
    }

    Ok(DecisionVariables { x, y, z })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            nodes: vec![
                Node { id: 0, ..Default::default() },
                Node { id: 1, ..Default::default() },
                Node { id: 2, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1, 2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_variable_initialization() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_var", &env)?;
        let instance = create_test_instance();
        let data = CluTopData::new(&instance);

        let vars = initialize_variables(&mut model, &instance, &data)?;

        assert_eq!(vars.x.len(), 1);
        assert_eq!(vars.x[0].len(), 3);
        assert_eq!(vars.y.len(), 1);
        assert_eq!(vars.y[0].len(), 3);
        assert_eq!(vars.z.len(), 1);

        Ok(())
    }
}
