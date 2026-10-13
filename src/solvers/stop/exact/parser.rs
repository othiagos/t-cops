use std::collections::HashSet;
use std::time::Duration;
use grb::prelude::*;
use crate::common::{
    instance::Instance,
    solution::{Route, Solution, SolverMetrics},
};
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::exact::variable::DecisionVariables;

pub fn parse_solution<'a>(
    model: &Model,
    variables: &DecisionVariables,
    instance: &'a Instance,
    data: &StopData,
    duration: Duration,
    metrics: SolverMetrics,
    status: Status,
) -> grb::Result<Solution<'a>> {
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();
    let depot = data.depot_id;

    let mut routes = Vec::new();

    for r in 0..num_vehicles {
        let mut succ_r = vec![None; num_nodes];
        for (i, succ) in succ_r.iter_mut().enumerate() {
            for j in 0..num_nodes {
                if i != j {
                    let val = model.get_obj_attr(attr::X, &variables.x[r][i][j])?;
                    if val > 0.5 {
                        *succ = Some(j);
                    }
                }
            }
        }

        if let Some(first_node) = succ_r[depot] {
            let mut path = vec![depot, first_node];
            let mut curr = first_node;
            let mut steps = 0;
            let max_steps = num_nodes + 2;

            while curr != depot && steps < max_steps {
                steps += 1;
                if let Some(next) = succ_r[curr] {
                    path.push(next);
                    curr = next;
                } else {
                    path.push(depot);
                    break;
                }
            }

            if path.len() > 2 {
                let cost = compute_path_cost(instance, &path);
                routes.push(Route {
                    vehicle_id: r,
                    path,
                    cost,
                });
            }
        }
    }

    let total_cost: f64 = routes.iter().map(|r| r.cost).sum();

    // Compute total score from all visited customer nodes
    let mut visited_customers = HashSet::new();
    for r in &routes {
        for &nid in &r.path {
            if nid != depot {
                visited_customers.insert(nid);
            }
        }
    }

    let total_score: f64 = visited_customers
        .iter()
        .map(|&nid| data.node_profits[nid])
        .sum();

    Ok(Solution {
        instance,
        duration,
        total_score,
        total_cost,
        routes,
        status: status.into(),
        solver: Some("Gurobi (STOP)".to_string()),
        best_bound: metrics.best_bound,
        gap: metrics.gap,
        explored_nodes: metrics.explored_nodes,
    })
}

fn compute_path_cost(instance: &Instance, path: &[usize]) -> f64 {
    if path.len() <= 1 {
        return 0.0;
    }
    let mut cost = 0.0;
    for i in 0..(path.len() - 1) {
        cost += instance.get_distance(path[i], path[i + 1]);
    }
    cost
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::stop::exact::variable::initialize_variables;

    #[test]
    fn test_stop_parse_solution_empty() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_stop_parse", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, ..Default::default() },
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
                Vehicle { id: 0, budget: 100.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };
        let data = StopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        model.optimize()?;
        let status = model.status()?;
        let metrics = SolverMetrics::default();

        let sol = parse_solution(&model, &vars, &instance, &data, Duration::from_secs(1), metrics, status)?;
        assert_eq!(sol.total_score, 0.0);
        Ok(())
    }
}
