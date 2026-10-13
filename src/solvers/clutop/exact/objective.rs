use grb::prelude::*;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::exact::variable::DecisionVariables;

pub fn set_objective(
    model: &mut Model,
    variables: &DecisionVariables,
    data: &CluTopData,
) -> grb::Result<()> {
    let obj_expr: grb::Expr = data
        .clusters
        .iter()
        .enumerate()
        .map(|(c_idx, cluster)| cluster.profit * variables.z[c_idx])
        .sum();

    model.set_objective(obj_expr, Maximize)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Subgroup, Vehicle, Instance};
    use crate::solvers::clutop::exact::variable::initialize_variables;

    #[test]
    fn test_set_objective() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_obj", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, ..Default::default() },
                Node { id: 1, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 25.0, node_ids: vec![1], ..Default::default() },
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

        set_objective(&mut model, &vars, &data)?;
        model.update()?;

        assert!(matches!(model.get_attr(attr::ModelSense)?, ModelSense::Maximize));
        Ok(())
    }
}
