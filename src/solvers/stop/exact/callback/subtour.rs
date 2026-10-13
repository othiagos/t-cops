#![allow(clippy::useless_conversion)]

use std::collections::VecDeque;
use grb::callback::{Callback, MIPSolCtx, Where};
use grb::prelude::*;
use crate::solvers::stop::exact::variable::DecisionVariables;

pub struct StopCallbackCache {
    pub x_vars: Vec<Vec<Var>>,
    pub x_arcs: Vec<Vec<(usize, usize)>>,
}

impl StopCallbackCache {
    pub fn new(variables: &DecisionVariables, num_vehicles: usize, num_nodes: usize) -> Self {
        let mut x_vars = Vec::with_capacity(num_vehicles);
        let mut x_arcs = Vec::with_capacity(num_vehicles);

        for r in 0..num_vehicles {
            let mut v_r = Vec::new();
            let mut a_r = Vec::new();
            for i in 0..num_nodes {
                for j in 0..num_nodes {
                    if i != j {
                        v_r.push(variables.x[r][i][j]);
                        a_r.push((i, j));
                    }
                }
            }
            x_vars.push(v_r);
            x_arcs.push(a_r);
        }

        Self { x_vars, x_arcs }
    }
}

pub struct StopSubtourCallback<'a> {
    pub variables: &'a DecisionVariables,
    pub cache: StopCallbackCache,
    pub num_vehicles: usize,
    pub num_nodes: usize,
    pub depot_id: usize,
}

impl<'a> StopSubtourCallback<'a> {
    pub fn new(
        variables: &'a DecisionVariables,
        num_vehicles: usize,
        num_nodes: usize,
        depot_id: usize,
    ) -> Self {
        let cache = StopCallbackCache::new(variables, num_vehicles, num_nodes);
        Self {
            variables,
            cache,
            num_vehicles,
            num_nodes,
            depot_id,
        }
    }

    fn handle_integer(&self, ctx: &mut MIPSolCtx) -> Result<(), anyhow::Error> {
        for r in 0..self.num_vehicles {
            let x_vals = ctx.get_solution(&self.cache.x_vars[r])?;

            let mut adj: Vec<Vec<usize>> = vec![Vec::new(); self.num_nodes];
            let mut in_degree: Vec<usize> = vec![0; self.num_nodes];
            let mut out_degree: Vec<usize> = vec![0; self.num_nodes];

            for (idx, &val) in x_vals.iter().enumerate() {
                if val > 0.5 {
                    let (i, j) = self.cache.x_arcs[r][idx];
                    adj[i].push(j);
                    out_degree[i] += 1;
                    in_degree[j] += 1;
                }
            }

            // Find all nodes reachable from the depot for vehicle r
            let mut reachable_from_depot = vec![false; self.num_nodes];
            let mut queue = VecDeque::new();
            reachable_from_depot[self.depot_id] = true;
            queue.push_back(self.depot_id);

            while let Some(u) = queue.pop_front() {
                for &v in &adj[u] {
                    if !reachable_from_depot[v] {
                        reachable_from_depot[v] = true;
                        queue.push_back(v);
                    }
                }
            }

            // Find connected components among customer nodes that are NOT reachable from depot
            let mut visited = reachable_from_depot.clone();
            for i in 0..self.num_nodes {
                if !visited[i] && (out_degree[i] > 0 || in_degree[i] > 0) {
                    let mut comp = Vec::new();
                    let mut comp_queue = VecDeque::new();
                    visited[i] = true;
                    comp_queue.push_back(i);

                    while let Some(u) = comp_queue.pop_front() {
                        comp.push(u);
                        // successors
                        for &v in &adj[u] {
                            if !visited[v] {
                                visited[v] = true;
                                comp_queue.push_back(v);
                            }
                        }
                        // predecessors
                        for p in 0..self.num_nodes {
                            if !visited[p] && adj[p].contains(&u) {
                                visited[p] = true;
                                comp_queue.push_back(p);
                            }
                        }
                    }

                    if comp.len() >= 2 && !comp.contains(&self.depot_id) {
                        // DFJ subtour elimination cut:
                        // sum_{u, v in comp, u != v} x_{r, u, v} <= |comp| - 1
                        let dfj_expr: grb::Expr = comp
                            .iter()
                            .flat_map(|&u| {
                                comp.iter().filter(move |&&v| u != v).map(move |&v| {
                                    1.0 * self.variables.x[r][u][v]
                                })
                            })
                            .sum();

                        let max_arcs = (comp.len() - 1) as f64;
                        ctx.add_lazy(c!(dfj_expr <= max_arcs))?;
                    }
                }
            }
        }

        Ok(())
    }
}

impl<'a> Callback for StopSubtourCallback<'a> {
    fn callback(&mut self, w: Where) -> Result<(), anyhow::Error> {
        if let Where::MIPSol(mut ctx) = w {
            self.handle_integer(&mut ctx)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Cluster, Instance, Node, Point3, Subgroup, Vehicle};
    use crate::solvers::stop::cluster::StopData;
    use crate::solvers::stop::exact::variable::initialize_variables;

    #[test]
    fn test_stop_subtour_callback_creation() -> grb::Result<()> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env("test_cb_creation", &env)?;
        let instance = Instance {
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 1.0, y: 1.0, z: 0.0 }, ..Default::default() },
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
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        };

        let data = StopData::new(&instance);
        let vars = initialize_variables(&mut model, &instance, &data)?;
        let cb = StopSubtourCallback::new(&vars, instance.vehicles.len(), instance.nodes.len(), data.depot_id);

        assert_eq!(cb.num_vehicles, 1);
        assert_eq!(cb.num_nodes, 2);
        assert_eq!(cb.depot_id, 0);
        assert_eq!(cb.cache.x_arcs[0].len(), 2); // (0,1) and (1,0)

        Ok(())
    }
}
