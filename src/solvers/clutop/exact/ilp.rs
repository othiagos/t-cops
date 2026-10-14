use std::time::Instant;
use grb::prelude::*;

use crate::common::{
    error::{SolverError, SolverErrorKind},
    instance::Instance,
    solution::{Solution, SolverMetrics},
};
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::exact::{
    callback::subtour::CluTopSubtourCallback,
    constraint,
    objective,
    parser,
    preprocess,
    variable::{self, DecisionVariables},
};
use crate::solvers::clutop::CluTopOptions;

pub struct Ilp<'a> {
    pub model: Model,
    variables: DecisionVariables,
    instance: &'a Instance,
    data: CluTopData,
}

impl<'a> Ilp<'a> {
    #[allow(dead_code)]
    pub fn q(&self) -> &Model {
        &self.model
    }

    pub fn new(instance: &'a Instance) -> grb::Result<Self> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env(&format!("{}_clutop", instance.name), &env)?;

        let data = CluTopData::new(instance);
        let variables = variable::initialize_variables(&mut model, instance, &data)?;

        Ok(Self {
            model,
            variables,
            instance,
            data,
        })
    }

    pub fn solve(mut self, options: &CluTopOptions) -> Result<Solution<'a>, SolverError> {
        let start_time = Instant::now();

        self.configure_solver(options)?;
        self.build_model()?;

        let mut callback = CluTopSubtourCallback::new(
            &self.variables,
            self.instance.vehicles.len(),
            self.instance.nodes.len(),
            self.data.depot_id,
        );

        self.model.optimize_with_callback(&mut callback)?;

        self.extract_solution(start_time)
    }

    pub fn configure_solver(&mut self, options: &CluTopOptions) -> Result<(), SolverError> {
        let threads = std::thread::available_parallelism()
            .map(|n| n.get() as i32)
            .unwrap_or(1);
        
        self.model.set_param(param::Threads, threads)?;
        self.model.set_param(param::Seed, 137)?;
        self.model.set_param(param::LazyConstraints, 1)?;

        if let Some(limit) = options.time_limit {
            self.model.set_param(param::TimeLimit, limit as f64)?;
        }

        if let Some(ref file) = options.gurobi_params_file {
            self.model.get_env_mut().read_params(file)?;
        }

        Ok(())
    }

    fn build_model(&mut self) -> Result<(), SolverError> {
        let model = &mut self.model;
        let variable = &self.variables;
        let instance = self.instance;
        let data = &self.data;

        constraint::unique_customer_visit(model, variable, instance, data)?;
        constraint::degree_conservation(model, variable, instance)?;
        constraint::budget(model, variable, instance)?;
        constraint::cluster_profit(model, variable, instance, data)?;
        constraint::cluster_coverage_cut(model, variable, instance, data)?;
        constraint::symmetry_breaking(model, variable, instance, data)?;

        preprocess::apply_preprocessing(model, variable, instance, data)?;

        objective::set_objective(model, variable, data)?;

        Ok(())
    }

    fn extract_solution(&self, start_time: Instant) -> Result<Solution<'a>, SolverError> {
        let status = self.model.status()?;

        match status {
            Status::Optimal | Status::TimeLimit | Status::IterationLimit => {
                let metrics = SolverMetrics {
                    best_bound: self.model.get_attr(attr::ObjBound).ok(),
                    gap: self.model.get_attr(attr::MIPGap).ok(),
                    explored_nodes: self.model.get_attr(attr::NodeCount).ok().map(|n| n as u64),
                };

                parser::parse_solution(
                    &self.model,
                    &self.variables,
                    self.instance,
                    &self.data,
                    start_time.elapsed(),
                    metrics,
                    status,
                )
                .map_err(|e| {
                    SolverError::new(
                        SolverErrorKind::Parser,
                        &format!("Error parsing CluTOP Gurobi results: {}", e),
                    )
                })
            }
            _ => Err(SolverError::new(
                SolverErrorKind::Solver,
                &format!("Gurobi failed to find a feasible solution. Status: {:?}", status),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            name: "ilp_clutop_test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 4.0, y: 0.0, z: 0.0 }, ..Default::default() },
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
                Vehicle { id: 0, budget: 20.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_ilp_clutop_solve() -> grb::Result<()> {
        let instance = create_test_instance();
        let ilp = Ilp::new(&instance)?;

        let options = CluTopOptions {
            mode: SolverMode::Exact,
            time_limit: Some(10),
            max_iterations: 10,
            gurobi_params_file: None,
        };

        let solution = ilp.solve(&options).expect("Solve should succeed");
        assert_eq!(solution.total_score, 10.0);
        assert!(!solution.routes.is_empty());
        assert_eq!(solution.routes[0].path.len(), 4); // 0 -> node -> node -> 0
        assert_eq!(solution.routes[0].path[0], 0);
        assert_eq!(solution.routes[0].path[3], 0);

        Ok(())
    }
}
