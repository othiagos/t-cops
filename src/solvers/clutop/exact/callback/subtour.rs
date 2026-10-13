#![allow(clippy::useless_conversion)]

use std::collections::VecDeque;
use grb::callback::{Callback, MIPSolCtx, Where};
use grb::prelude::*;
use crate::solvers::clutop::exact::variable::DecisionVariables;

pub struct CluTopCallbackCache {
    pub x_vars: Vec<Vec<Var>>,
    pub x_edges: Vec<Vec<(usize, usize)>>,
}

impl CluTopCallbackCache {
    pub fn new(variables: &DecisionVariables, num_vehicles: usize, num_nodes: usize) -> Self {
        let mut x_vars = Vec::with_capacity(num_vehicles);
        let mut x_edges = Vec::with_capacity(num_vehicles);

        for r in 0..num_vehicles {
            let num_edges = num_nodes * (num_nodes.saturating_sub(1)) / 2;
            let mut v_list = Vec::with_capacity(num_edges);
            let mut e_list = Vec::with_capacity(num_edges);

            for i in 0..num_nodes {
                for j in (i + 1)..num_nodes {
                    v_list.push(variables.x[r][i][j]);
                    e_list.push((i, j));
                }
            }

            x_vars.push(v_list);
            x_edges.push(e_list);
        }

        Self { x_vars, x_edges }
    }
}

pub struct CluTopSubtourCallback<'a> {
    pub variables: &'a DecisionVariables,
    pub cache: CluTopCallbackCache,
    pub num_vehicles: usize,
    pub num_nodes: usize,
    pub depot_id: usize,
}

impl<'a> CluTopSubtourCallback<'a> {
    pub fn new(
        variables: &'a DecisionVariables,
        num_vehicles: usize,
        num_nodes: usize,
        depot_id: usize,
    ) -> Self {
        let cache = CluTopCallbackCache::new(variables, num_vehicles, num_nodes);
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

            // Build adjacency list for active edges in vehicle r
            let mut adj: Vec<Vec<usize>> = vec![Vec::new(); self.num_nodes];
            let mut active_count = 0;

            for (idx, &val) in x_vals.iter().enumerate() {
                if val > 0.5 {
                    let (i, j) = self.cache.x_edges[r][idx];
                    adj[i].push(j);
                    adj[j].push(i);
                    active_count += 1;
                }
            }

            if active_count == 0 {
                continue;
            }

            // Find connected components
            let mut visited = vec![false; self.num_nodes];
            let mut components = Vec::new();

            for i in 0..self.num_nodes {
                if !visited[i] && !adj[i].is_empty() {
                    let mut comp = Vec::new();
                    let mut queue = VecDeque::new();
                    visited[i] = true;
                    queue.push_back(i);

                    while let Some(u) = queue.pop_front() {
                        comp.push(u);
                        for &v in &adj[u] {
                            if !visited[v] {
                                visited[v] = true;
                                queue.push_back(v);
                            }
                        }
                    }

                    components.push(comp);
                }
            }

            // Any component that does NOT contain the depot is a subtour
            for comp in components {
                if !comp.contains(&self.depot_id) && comp.len() >= 2 {
                    // DFJ cut: edges in comp <= |comp| - 1
                    let dfj_expr: grb::Expr = comp
                        .iter()
                        .flat_map(|&u| {
                            comp.iter().filter(move |&&v| u < v).map(move |&v| {
                                1.0 * self.variables.x[r][u][v]
                            })
                        })
                        .sum();

                    let max_edges = (comp.len() - 1) as f64;
                    ctx.add_lazy(c!(dfj_expr <= max_edges))?;

                    // Cut-set constraint: edges between comp and V \ comp >= 2 * y[r][first]
                    let u0 = comp[0];
                    let cut_set_expr: grb::Expr = comp
                        .iter()
                        .flat_map(|&u| {
                            (0..self.num_nodes).filter(|n| !comp.contains(n)).map(move |v| {
                                1.0 * self.variables.x[r][u.min(v)][u.max(v)]
                            })
                        })
                        .sum();

                    let y_trigger = self.variables.y[r][u0];
                    ctx.add_lazy(c!(cut_set_expr >= 2.0 * y_trigger))?;
                }
            }
        }

        Ok(())
    }
}

impl<'a> Callback for CluTopSubtourCallback<'a> {
    fn callback(&mut self, w: Where) -> Result<(), anyhow::Error> {
        if let Where::MIPSol(mut ctx) = w {
            self.handle_integer(&mut ctx)?;
        }
        Ok(())
    }
}
