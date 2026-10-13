use crate::common::instance::Instance;

pub fn calculate_path_cost(instance: &Instance, path: &[usize]) -> f64 {
    if path.len() <= 1 {
        return 0.0;
    }
    let mut cost = 0.0;
    for i in 0..(path.len() - 1) {
        cost += instance.get_distance(path[i], path[i + 1]);
    }
    cost
}

/// 2-opt local search on a single route [0, v1, v2, ..., vk, 0]
pub fn two_opt(instance: &Instance, path: &mut [usize]) -> f64 {
    if path.len() <= 3 {
        return calculate_path_cost(instance, path);
    }

    let mut improved = true;
    let mut current_cost = calculate_path_cost(instance, path);

    while improved {
        improved = false;
        let n = path.len();

        for i in 1..(n - 2) {
            for j in (i + 1)..(n - 1) {
                // Check edge (i-1, i) and (j, j+1) vs (i-1, j) and (i, j+1)
                let a = path[i - 1];
                let b = path[i];
                let c = path[j];
                let d = path[j + 1];

                let old_dist = instance.get_distance(a, b) + instance.get_distance(c, d);
                let new_dist = instance.get_distance(a, c) + instance.get_distance(b, d);

                if new_dist + 1e-6 < old_dist {
                    path[i..=j].reverse();
                    current_cost -= old_dist - new_dist;
                    improved = true;
                    break;
                }
            }
            if improved {
                break;
            }
        }
    }

    current_cost
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::instance::{Metric, Node, Point3};

    #[test]
    fn test_two_opt_untangles() {
        let instance = Instance {
            metric: Metric::Euc2d,
            nodes: vec![
                Node { id: 0, point: Point3 { x: 0.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 1, point: Point3 { x: 0.0, y: 1.0, z: 0.0 }, ..Default::default() },
                Node { id: 2, point: Point3 { x: 1.0, y: 0.0, z: 0.0 }, ..Default::default() },
                Node { id: 3, point: Point3 { x: 1.0, y: 1.0, z: 0.0 }, ..Default::default() },
            ],
            ..Default::default()
        };

        // Tangled path: 0 -> 3 -> 1 -> 2 -> 0
        let mut path = vec![0, 3, 1, 2, 0];
        let initial_cost = calculate_path_cost(&instance, &path);
        let opt_cost = two_opt(&instance, &mut path);

        assert!(opt_cost <= initial_cost);
        assert_eq!(path[0], 0);
        assert_eq!(*path.last().unwrap(), 0);
    }
}
