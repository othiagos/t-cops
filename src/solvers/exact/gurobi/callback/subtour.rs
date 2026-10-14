use grb::callback::{Callback, MIPNodeCtx, MIPSolCtx, Where};

use crate::common::instance::Instance;
use crate::solvers::exact::gurobi::callback::utils::CutTuple;
use crate::solvers::exact::gurobi::ilp::DecisionVariables;

use super::cache::CallbackCache;
use super::{io, separation, utils};

pub struct SubtourCallback<'a> {
    pub variables: &'a DecisionVariables,
    pub instance: &'a Instance,
    pub enable_fractional_cuts: bool,
    cache: Option<CallbackCache>,
}

impl<'a> SubtourCallback<'a> {
    pub fn new(
        variables: &'a DecisionVariables,
        instance: &'a Instance,
        enable_fractional_cuts: bool,
    ) -> Self {
        Self {
            variables,
            instance,
            enable_fractional_cuts,
            cache: None,
        }
    }

    fn handle_fractional(&self, ctx: &mut MIPNodeCtx) -> Result<(), anyhow::Error> {
        if ctx.status()? != grb::Status::Optimal {
            return Ok(());
        }

        let all_cuts = self.gather_all_fractional_cuts(ctx)?;

        if all_cuts.is_empty() {
            return Ok(());
        }

        let w_cuts = utils::filter_orthogonal_cuts(all_cuts);

        let num_nodes = self.instance.nodes.len();
        
        for (k, tour, _, src) in w_cuts {
            io::apply_fractional_cuts(k, vec![(tour, src)], ctx, self.variables, num_nodes)?;
        }

        Ok(())
    }

    fn gather_all_fractional_cuts(&self, ctx: &MIPNodeCtx) -> Result<Vec<CutTuple>, anyhow::Error> {
        let num_nodes = self.instance.nodes.len();
        let cache = self.cache.as_ref().unwrap();
        let mut all_cuts = Vec::new();

        for k in 0..self.instance.vehicles.len() {
            let (frac_edges, y_vals) = io::get_fractional_graph(k, ctx, cache)?;
            
            let bad_tours = separation::find_fractional_subtours(
                num_nodes,
                self.instance.vehicles[k].start_node_id, 
                &frac_edges,
                &y_vals,
            );
            
            for (tour, violation, src) in bad_tours {
                all_cuts.push((k, tour, violation, src));
            }
        }

        Ok(all_cuts)
    }

    fn handle_integer(&self, ctx: &mut MIPSolCtx) -> Result<(), anyhow::Error> {
        let num_nodes = self.instance.nodes.len();
        let cache = self.cache.as_ref().unwrap();

        for k in 0..self.instance.vehicles.len() {
            let active_edges = io::get_active_edges(k, ctx, cache)?;
            if active_edges.is_empty() {
                continue;
            }

            let bad_tours = separation::find_invalid_subtours(
                num_nodes,
                self.instance.vehicles[k].start_node_id,
                self.instance.vehicles[k].end_node_id,
                &active_edges,
            );

            if !bad_tours.is_empty() {
                io::apply_cut_set_constraints(k, bad_tours, ctx, self.variables, num_nodes)?;
            }
        }

        Ok(())
    }
}

impl<'a> Callback for SubtourCallback<'a> {
    fn callback(&mut self, w: Where) -> Result<(), anyhow::Error> {
        if self.cache.is_none() {
            self.cache = Some(CallbackCache::new(self.variables, self.instance));
        }

        match w {
            Where::MIPNode(mut ctx) if self.enable_fractional_cuts => self.handle_fractional(&mut ctx),
            Where::MIPSol(mut ctx) => self.handle_integer(&mut ctx),
            _ => Ok(()),
        }
    }
}