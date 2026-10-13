use std::time::Duration;
use grb::prelude::*;
use crate::common::{
    instance::Instance,
    solution::{Route, Solution, SolverMetrics},
};
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::exact::variable::DecisionVariables;

pub fn parse_solution<'a>(
    model: &Model,
    variables: &DecisionVariables,
    instance: &'a Instance,
    data: &CluTopData,
    duration: Duration,
    metrics: SolverMetrics,
    status: Status,
) -> grb::Result<Solution<'a>> {
    let mut routes: Vec<Route> = Vec::new();
    let num_vehicles = instance.vehicles.len();
    let num_nodes = instance.nodes.len();
    let depot = data.depot_id;

    for r in 0..num_vehicles {
        let is_used = model.get_obj_attr(attr::X, &variables.y[r][depot])? > 0.5;

        if !is_used {
            continue;
        }

        // Build adjacency list for vehicle r
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); num_nodes];
        for i in 0..num_nodes {
            for j in (i + 1)..num_nodes {
                let val = model.get_obj_attr(attr::X, &variables.x[r][i][j])?;
                if val > 0.5 {
                    adj[i].push(j);
                    adj[j].push(i);
                }
            }
        }

        if adj[depot].is_empty() {
            continue;
        }

        // Trace the cycle starting at depot
        let mut path: Vec<usize> = Vec::new();
        path.push(depot);

        let mut prev = depot;
        let mut curr = adj[depot][0];
        path.push(curr);

        let mut steps = 0;
        let max_steps = num_nodes + 2;

        while curr != depot && steps < max_steps {
            steps += 1;
            let next_candidates: Vec<usize> = adj[curr].iter().copied().filter(|&v| v != prev).collect();
            if next_candidates.is_empty() {
                // If dead-end, return to depot to close
                if adj[curr].contains(&depot) {
                    path.push(depot);
                }
                break;
            }
            let next = next_candidates[0];
            path.push(next);
            prev = curr;
            curr = next;
        }

        // Calculate cost
        let mut route_cost = 0.0;
        for i in 0..(path.len().saturating_sub(1)) {
            route_cost += instance.get_distance(path[i], path[i + 1]);
        }

        routes.push(Route {
            vehicle_id: r,
            path,
            cost: route_cost,
        });
    }

    let total_score: f64 = data
        .clusters
        .iter()
        .enumerate()
        .filter_map(|(c_idx, cluster)| {
            let val = model.get_obj_attr(attr::X, &variables.z[c_idx]).ok()?;
            if val >= 0.5 {
                Some(cluster.profit)
            } else {
                None
            }
        })
        .sum();

    let total_cost: f64 = routes.iter().map(|r| r.cost).sum();

    Ok(Solution {
        instance,
        duration,
        total_score,
        total_cost,
        routes,
        status: status.into(),
        solver: Some("Gurobi (CluTOP)".to_string()),
        best_bound: metrics.best_bound,
        gap: metrics.gap,
        explored_nodes: metrics.explored_nodes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::clutop::exact::variable::initialize_variables;

    fn create_test_instance() -> Instance {
        Instance {
            name: "parser_test".to_string(),
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 4.0, y: 0.0, z: 0.0 }, ..Default::default() },
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
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_parse_solution_empty() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_parse", &env)?;
        let instance = create_test_instance();
        let data = CluTopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;

        model.optimize()?;
        let status = model.status()?;

        let metrics = SolverMetrics::default();
        let sol = parse_solution(&model, &vars, &instance, &data, Duration::from_secs(1), metrics, status)?;

        assert_eq!(sol.total_score, 0.0);
        Ok(())
    }
}
