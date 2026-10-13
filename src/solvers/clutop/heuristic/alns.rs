use std::time::Instant;
use rand::{thread_rng, Rng};

use crate::common::instance::Instance;
use crate::solvers::clutop::cluster::CluTopData;
use crate::solvers::clutop::heuristic::giant_tour::construct_giant_tour;
use crate::solvers::clutop::heuristic::operators::{
    destroy, repair, HeuristicState, InsertionOperatorType,
};
use crate::solvers::clutop::heuristic::split::split_procedure;
use crate::solvers::clutop::CluTopOptions;

const SIGMA_BEST: f64 = 33.0;
const SIGMA_BETTER: f64 = 9.0;
const SIGMA_WORSE: f64 = 1.0;
const REACTION_FACTOR: f64 = 0.2;

pub fn solve_alns(
    instance: &Instance,
    data: &CluTopData,
    options: &CluTopOptions,
) -> HeuristicState {
    let start_time = Instant::now();
    let mut rng = thread_rng();

    // 1. Initial greedy solution: repair with Best insertion from empty state
    let mut current_state = HeuristicState::new(instance, data.depot_id);
    repair(
        &mut current_state,
        InsertionOperatorType::Best,
        instance,
        data,
        &mut rng,
    );

    let mut best_state = current_state.clone();

    // Operator weights
    let operators = [
        InsertionOperatorType::Best,
        InsertionOperatorType::RandomBest,
        InsertionOperatorType::Regret2,
    ];
    let mut weights = vec![1.0; operators.len()];
    let mut scores = vec![0.0; operators.len()];
    let mut counts = vec![0; operators.len()];

    let max_iterations = options.max_iterations.max(10);
    let mut d_max = 3.min(data.clusters.len().max(1));

    let mut iter_without_improvement = 0;
    let max_iter_without_improvement = 50.min(max_iterations);

    for iter in 0..max_iterations {
        if let Some(tl) = options.time_limit && start_time.elapsed().as_secs() >= tl {
            break;
        }

        if iter_without_improvement >= max_iter_without_improvement {
            break;
        }

        let mut trial_state = current_state.clone();

        // 2. Destroy
        destroy(&mut trial_state, d_max, instance, data, &mut rng);

        // 3. Select operator
        let op_idx = select_operator(&weights, &mut rng);
        let chosen_op = operators[op_idx];
        counts[op_idx] += 1;

        // 4. Repair
        repair(&mut trial_state, chosen_op, instance, data, &mut rng);

        // 5. Evaluate
        let prev_score = current_state.total_score;
        let trial_score = trial_state.total_score;

        if trial_score > best_state.total_score + 1e-6 {
            scores[op_idx] += SIGMA_BEST;
            best_state = trial_state.clone();
            current_state = trial_state;
            d_max = (d_max + 1).min(data.clusters.len().max(1));
            iter_without_improvement = 0;
        } else if trial_score > prev_score + 1e-6 {
            scores[op_idx] += SIGMA_BETTER;
            current_state = trial_state;
            d_max = (d_max + 1).min(data.clusters.len().max(1));
            iter_without_improvement += 1;
        } else {
            scores[op_idx] += SIGMA_WORSE;
            d_max = 3.min(data.clusters.len().max(1));
            iter_without_improvement += 1;
        }

        // Periodic weight adjustment
        if (iter + 1) % 10 == 0 {
            for i in 0..operators.len() {
                if counts[i] > 0 {
                    let avg = scores[i] / (counts[i] as f64);
                    weights[i] = (1.0 - REACTION_FACTOR) * weights[i] + REACTION_FACTOR * avg;
                    weights[i] = weights[i].max(0.1);
                    scores[i] = 0.0;
                    counts[i] = 0;
                }
            }
        }

        // Periodic Split call (every 20 iterations or if improvement)
        if (iter + 1) % 20 == 0 {
            let gt = construct_giant_tour(&best_state, data, instance.nodes.len(), &mut rng);
            let split_result = split_procedure(&gt, instance, data, best_state.clone());
            
            if split_result.total_score > best_state.total_score + 1e-6 {
                best_state = split_result.clone();
                current_state = split_result;
                iter_without_improvement = 0;
            }
        }
    }

    // Final Split on best solution
    let gt = construct_giant_tour(&best_state, data, instance.nodes.len(), &mut rng);
    let split_result = split_procedure(&gt, instance, data, best_state.clone());

    if split_result.total_score > best_state.total_score + 1e-6 {
        best_state = split_result;
    }

    best_state
}

fn select_operator<R: Rng>(weights: &[f64], rng: &mut R) -> usize {
    let total: f64 = weights.iter().sum();

    if total <= 0.0 {
        return 0;
    }

    let mut r = rng.gen_range(0.0..total);

    for (i, &w) in weights.iter().enumerate() {
        if r <= w {
            return i;
        }
        r -= w;
    }
    weights.len() - 1
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
    fn test_solve_alns() {
        let instance = create_test_instance();
        let data = CluTopData::new(&instance);
        let options = CluTopOptions {
            mode: SolverMode::Heuristic,
            time_limit: Some(2),
            max_iterations: 15,
            gurobi_params_file: None,
        };

        let res = solve_alns(&instance, &data, &options);
        assert_eq!(res.total_score, 30.0);
        assert!(res.is_feasible(&instance));
    }
}
