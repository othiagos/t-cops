use std::time::Instant;
use rand::{thread_rng, Rng};

use crate::common::instance::Instance;
use crate::solvers::stop::cluster::StopData;
use crate::solvers::stop::heuristic::destroy::{destroy, RemovalOperatorType, StopSolution};
use crate::solvers::stop::heuristic::initial::build_initial_solution;
use crate::solvers::stop::heuristic::insert::{repair, InsertionOperatorType};
use crate::solvers::stop::StopOptions;

pub fn solve_lns(
    instance: &Instance,
    data: &StopData,
    options: &StopOptions,
) -> StopSolution {
    let start_time = Instant::now();
    let mut rng = thread_rng();

    // 1. Initial solution
    let mut current_sol = build_initial_solution(instance, data);
    let mut best_sol = current_sol.clone();

    // 2. Initial temperature for Simulated Annealing (Eq. 20)
    let mu = 0.11;
    let initial_score = current_sol.total_score.max(1.0);
    let mut t = (-mu * initial_score) / (0.5_f64.ln());

    let max_iterations = options.max_iterations.max(50);
    // Cooling rate alpha computed so temperature cools over max_iterations
    let alpha = (0.001_f64 / t.max(0.001)).powf(1.0 / (max_iterations as f64)).clamp(0.9, 0.99994);

    // Operator pools and weights (Table 2 of paper)
    let removal_ops = [
        (RemovalOperatorType::Random, 0.19),
        (RemovalOperatorType::WorstCost, 0.13),
        (RemovalOperatorType::WorstProfit, 0.38),
        (RemovalOperatorType::WorstProfitOverCost, 0.20),
        (RemovalOperatorType::Related, 0.09),
    ];

    let insertion_ops = [
        (InsertionOperatorType::RandomOrder, 0.29),
        (InsertionOperatorType::ProfitBased, 0.36),
        (InsertionOperatorType::ProximityBased, 0.12),
        (InsertionOperatorType::Cheapest, 0.07),
        (InsertionOperatorType::DynamicProfitOverCost, 0.02),
        (InsertionOperatorType::Regret2, 0.14),
    ];

    let mut iter_without_improvement = 0;
    let max_iter_without_improvement = 200.min(max_iterations);

    for _iter in 0..max_iterations {
        if let Some(tl) = options.time_limit && start_time.elapsed().as_secs() >= tl {
            break;
        }

        if iter_without_improvement >= max_iter_without_improvement {
            break;
        }

        let mut trial_sol = current_sol.clone();

        // Select operators via roulette wheel
        let rem_op = select_by_weight(&removal_ops, &mut rng);
        let ins_op = select_by_weight(&insertion_ops, &mut rng);

        // Destroy + Repair
        destroy(&mut trial_sol, rem_op, instance, data, &mut rng);
        repair(&mut trial_sol, ins_op, instance, data, &mut rng);

        // Simulated Annealing acceptance
        let delta_f = trial_sol.total_score - current_sol.total_score;
        let accept = if delta_f >= 0.0 {
            true
        } else {
            let prob = (delta_f / t.max(1e-6)).exp();
            rng.gen_range(0.0..1.0) < prob
        };

        if accept {
            current_sol = trial_sol;
            if current_sol.total_score > best_sol.total_score + 1e-6 {
                best_sol = current_sol.clone();
                iter_without_improvement = 0;
            } else {
                iter_without_improvement += 1;
            }
        } else {
            iter_without_improvement += 1;
        }

        t *= alpha;
    }

    best_sol
}

fn select_by_weight<T: Copy, R: Rng>(ops: &[(T, f64)], rng: &mut R) -> T {
    let total: f64 = ops.iter().map(|o| o.1).sum();
    let mut r = rng.gen_range(0.0..total);
    for &(op, weight) in ops {
        if r <= weight {
            return op;
        }
        r -= weight;
    }
    ops.last().unwrap().0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::SolverMode;
    use crate::common::instance::{Cluster, Metric, Node, Point3, Subgroup, Vehicle};

    fn create_test_instance() -> Instance {
        Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 1.0, y: 0.0, z: 0.0 }, ..Default::default() },
            ],
            subgroups: vec![
                Subgroup { id: 0, profit: 0.0, node_ids: vec![0], ..Default::default() },
                Subgroup { id: 1, profit: 10.0, node_ids: vec![1], ..Default::default() },
                Subgroup { id: 2, profit: 20.0, node_ids: vec![2], ..Default::default() },
            ],
            clusters: vec![
                Cluster { id: 0, subgroup_ids: vec![0] },
                Cluster { id: 1, subgroup_ids: vec![1] },
                Cluster { id: 2, subgroup_ids: vec![2] },
            ],
            vehicles: vec![
                Vehicle { id: 0, budget: 50.0, start_node_id: 0, end_node_id: 0 },
            ],
            ..Default::default()
        }
    }

    #[test]
    fn test_solve_lns() {
        let instance = create_test_instance();
        let data = StopData::new(&instance);
        let options = StopOptions {
            mode: SolverMode::Heuristic,
            time_limit: Some(2),
            max_iterations: 20,
            gurobi_params_file: None,
        };

        let best = solve_lns(&instance, &data, &options);
        assert_eq!(best.total_score, 30.0);
    }
}
