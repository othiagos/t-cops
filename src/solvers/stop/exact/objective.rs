use grb::prelude::*;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::exact::variable::DecisionVariables;

pub fn set_objective(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &StopData,
    num_nodes: usize,
) -> grb::Result<()> {
    let num_vehicles = variables.x.len();
    let obj_expr: grb::Expr = (0..num_nodes)
        .filter(|&j| j != data.depot_id && data.node_profits[j] > 0.0)
        .flat_map(|j| {
            let profit_j = data.node_profits[j];
            (0..num_vehicles).flat_map(move |r| {
                (0..num_nodes)
                    .filter(move |&i| i != j)
                    .map(move |i| profit_j * variables.x[r][i][j])
            })
        })
        .sum();

    model.set_objective(obj_expr, Maximize)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Instance, Node, Subgroup, Vehicle};
    use crate::solvers::stop::exact::variable::initialize_variables;

    #[test]
    fn test_set_objective() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_stop_obj", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, ..Default::default() },
                Node { id: 1, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 50.0, node_ids: vec![1], ..Default::default() },
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

        set_objective(&mut model, &vars, &data, instance.nodes.len())?;
        model.update()?;

        assert!(matches!(model.get_attr(attr::ModelSense)?, ModelSense::Maximize));
        Ok(())
    }
}
