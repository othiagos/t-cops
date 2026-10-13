#![allow(dead_code)]

pub mod cluster;
pub mod exact;
pub mod heuristic;

use crate::cli::SolverMode;
use crate::common::{error::SolverError, instance::Instance, solution::Solution};

#[derive(Debug, Clone)]
pub struct CluTopOptions {
    pub mode: SolverMode,
    pub time_limit: Option<u64>,
    pub max_iterations: usize,
    pub gurobi_params_file: Option<String>,
}

pub fn solve<'a>(
    instance: &'a Instance,
    options: &CluTopOptions,
) -> Result<Solution<'a>, SolverError> {
    match options.mode {
        SolverMode::Exact => exact::solve(instance, options),
        SolverMode::Heuristic => heuristic::solve(instance, options),
    }
}
