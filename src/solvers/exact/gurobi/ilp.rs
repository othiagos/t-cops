use std::time::Instant;

use grb::prelude::*;

use crate::{
    common::{
        error::{SolverError, SolverErrorKind},
        instance::Instance,
        solution::{Solution, SolverMetrics},
    },
    solvers::exact::gurobi::{
        callback::subtour::SubtourCallback,
        constraint,
        objective, 
        parser,
        variable
    }
};

use crate::cli::Cli;
pub use variable::DecisionVariables;

pub struct Ilp<'a> {
    pub model: Model,
    variables: DecisionVariables,
    instance: &'a Instance,
}

impl<'a> Ilp<'a> {
    #[allow(dead_code)]
    pub fn model(&self) -> &Model {
        &self.model
    }

    pub fn new(instance: &'a Instance) -> grb::Result<Self> {
        let env = Env::new("gurobi.log")?;
        let mut model = Model::with_env(&instance.name, &env)?;

        let x = variable::initialize_x(&mut model, instance)?;
        let y = variable::initialize_y(&mut model, instance)?;
        let z = variable::initialize_z(&mut model, instance)?;
        let w = variable::initialize_w(&mut model, instance)?;

        let variables = DecisionVariables { x, y, z, w };

        Ok(Self {
            model,
            variables,
            instance,
        })
    }

    pub fn solve(mut self, args: &Cli) -> Result<Solution<'a>, SolverError> {
        let start_time = Instant::now();

        self.configure_solver(args)?;

        self.build_model()?;

        let mut callback = SubtourCallback::new(&self.variables, self.instance);
        self.model.optimize_with_callback(&mut callback)?;

        self.extract_solution(start_time)
    }

    pub fn configure_solver(&mut self, args: &Cli) -> Result<(), SolverError> {
        let threads = std::thread::available_parallelism()
            .map(|n| n.get() as i32)
            .unwrap_or(1);
        
        self.model.set_param(param::Threads, threads)?;
        self.model.set_param(param::Seed, 137)?;
        self.model.set_param(param::LazyConstraints, 1)?;
        
        if let Some(limit) = args.time_limit {
            self.model.set_param(param::TimeLimit, limit as f64)?;
        }

        if let Some(file) = &args.gurobi_params_file {
            self.model.get_env_mut().read_params(file)?;
        } 

        Ok(())
    }

    fn build_model(&mut self) -> Result<(), SolverError> {
        let model = &mut self.model;
        let variable = &self.variables;
        let instance = self.instance;

        constraint::intermediate_flow_conservation(model, variable, instance)?;
        constraint::unique_visit(model, variable, instance)?;
        constraint::depot_flow(model, variable, instance)?;
        constraint::depot_blocks(model, variable, instance)?;
        constraint::logical_physical_z_leq_y(model, variable, instance)?;
        constraint::logical_physical_y_leq_z(model, variable, instance)?;
        constraint::cluster(model, variable, instance)?;
        constraint::budget(model, variable, instance)?;
        constraint::logical_visit(model, variable, instance)?;

        objective::set_objective(model, variable, instance)?;

        Ok(())
    }

    fn extract_solution(&self, start_time: Instant) -> Result<Solution<'a>, SolverError> {
        let status = self.model.status()?;

        match status {
            Status::Optimal | Status::TimeLimit | Status::IterationLimit => {
                self.handle_successful_solution(start_time, status)
            }
            _ => {
                self.handle_failed_solution(status)
            }
        }
    }

    fn handle_successful_solution(&self, start_time: Instant, status: Status) -> Result<Solution<'a>, SolverError> {
        let metrics = SolverMetrics {
            best_bound: self.model.get_attr(attr::ObjBound).ok(),
            gap: self.model.get_attr(attr::MIPGap).ok(),
            explored_nodes: self.model.get_attr(attr::NodeCount).ok().map(|n| n as u64),
        };

        parser::parse_solution(
            &self.model,
            &self.variables,
            self.instance,
            start_time.elapsed(),
            metrics,
            status,
        )
        .map_err(|e| {
            SolverError::new(
                SolverErrorKind::Parser,
                &format!("Error parsing Gurobi results: {}", e),
            )
        })
    }

    fn handle_failed_solution(&self, status: Status) -> Result<Solution<'a>, SolverError> {
        Err(SolverError::new(
            SolverErrorKind::Solver,
            &format!("Gurobi failed to find a solution. Status: {:?}", status),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use std::path::PathBuf;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            name: "ilp_test".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, parent_subgroup_ids: HashSet::new() },
                Node { id: 1, point: Point3 { x: 0.0, y: 3.0, z: 0.0 }, parent_subgroup_ids: HashSet::from([0]) },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 10.0, node_ids: vec![1], parent_cluster_ids: HashSet::from([0]) },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 20.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_ilp_creation_and_solving() -> grb::Result<()> {
        let instance = create_test_instance();
        let ilp = Ilp::new(&instance)?;

        let cli = Cli {
            input: PathBuf::from("dummy.tcops"),
            mode: SolverMode::Exact,
            library: None,
            max_iterations: 10,
            max_shaking_intensity: 5,
            show: false,
            save: false,
            time_limit: Some(5),
            folder_result: "./result".to_string(),
            custom_result_name: None,
            gurobi_params_file: None,
            #[cfg(feature = "lib_good_lp")]
            solver: None,
        };

        let solution = ilp.solve(&cli).expect("ILP solve should succeed");
        assert_eq!(solution.total_score, 10.0);
        assert_eq!(solution.routes[0].path, vec![0, 1, 0]);

        Ok(())
    }
}
