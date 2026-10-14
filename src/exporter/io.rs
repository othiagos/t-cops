use std::collections::HashSet;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use serde::Serialize;

use crate::cli::Cli;
use crate::common::instance::Metric;
use crate::common::solution::Solution;

#[derive(Serialize)]
struct Node {
    id: usize,
    x: f64,
    y: f64,
    z: f64,
    parent_subgroup_ids: Vec<usize>,
}

#[derive(Serialize)]
struct Subgroup {
    id: usize,
    profit: f64,
    node_ids: Vec<usize>,
}

#[derive(Serialize)]
struct Cluster {
    id: usize,
    subgroup_ids: Vec<usize>,
}

#[derive(Serialize)]
struct Vehicle {
    id: usize,
    budget: f64,
    start_node_id: usize,
    end_node_id: usize,
}

#[derive(Serialize)]
struct Route {
    vehicle_id: usize,
    path: Vec<usize>,
}

#[derive(Serialize)]
struct JsonSolution {
    instance_name: String,
    mode: String,
    solver: Option<String>,
    elapsed_time_sec: f64,
    status: String,
    total_cost: f64,
    total_score: f64,
    best_bound: Option<f64>,
    gap: Option<f64>,
    explored_nodes: Option<u64>,
    vehicles_used_ids: Vec<usize>,
    clusters_visited_ids: Vec<usize>,
    subgroups_visited_ids: Vec<usize>,
    nodes: Vec<Node>,
    subgroups: Vec<Subgroup>,
    clusters: Vec<Cluster>,
    vehicles: Vec<Vehicle>,
    routes: Vec<Route>,
}

pub fn export_solution_to_json(solution: &Solution, args: &Cli) -> Option<String> {
    let instance_name = solution.instance.name.clone();
    let mode = get_mode(solution);
    let elapsed_time_sec = solution.duration.as_secs_f64();
    let total_cost = solution.total_cost;
    let total_score = solution.total_score;
    let vehicles_used_ids = get_vehicles_used(solution);
    let clusters_visited_ids = get_clusters_visited(solution);
    let subgroups_visited_ids = get_subgroups_visited(solution);
    let status = solution.status.clone().to_string();
    let solver = solution.solver.clone();
    let best_bound = solution.best_bound;
    let gap = solution.gap;
    let explored_nodes = solution.explored_nodes;
    let nodes = get_node(solution);
    let subgroups = get_subgroup(solution);
    let clusters = get_cluster(solution);
    let vehicles = get_vehicles(solution);
    let routes = get_routes(solution);

    let json_solution = JsonSolution {
        instance_name,
        mode,
        elapsed_time_sec,
        status,
        total_cost,
        total_score,
        vehicles_used_ids,
        clusters_visited_ids,
        subgroups_visited_ids,
        nodes,
        subgroups,
        clusters,
        vehicles,
        routes,
        solver,
        best_bound,
        gap,
        explored_nodes,
    };

    let path = if let Some(name) = &args.custom_result_name {
        format!("{}/{}.json", args.folder_result, name)
    } else {
        format!("{}/{}.json", args.folder_result, solution.instance.name)
    };

    if let Some(parent) = Path::new(&path).parent()
        && !parent.exists()
        && let Err(e) = fs::create_dir_all(parent)
    {
        eprintln!("Failed to create result folder: {}", e);
        return None;
    }

    let file = match File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Failed to create JSON file: {}", e);
            return None;
        }
    };

    if let Err(e) = serde_json::to_writer_pretty(BufWriter::new(file), &json_solution) {
        eprintln!("Failed to write JSON: {}", e);
        return None;
    }

    Some(path)
}

fn get_mode(solution: &Solution) -> String {
    match solution.instance.metric {
        Metric::Euc2d | Metric::Man2d => "2d".to_string(),
        Metric::Euc3d | Metric::Man3d => "3d".to_string(),
    }
}

fn get_node(solution: &Solution) -> Vec<Node> {
    solution
        .instance
        .nodes
        .iter()
        .map(|n| Node {
            id: n.id,
            x: n.point.x,
            y: n.point.y,
            z: n.point.z,
            parent_subgroup_ids: n.parent_subgroup_ids.iter().copied().collect(),
        })
        .collect()
}

fn get_subgroup(solution: &Solution) -> Vec<Subgroup> {
    solution
        .instance
        .subgroups
        .iter()
        .map(|s| Subgroup {
            id: s.id,
            profit: s.profit,
            node_ids: s.node_ids.clone(),
        })
        .collect()
}

fn get_cluster(solution: &Solution) -> Vec<Cluster> {
    solution
        .instance
        .clusters
        .iter()
        .map(|c| Cluster {
            id: c.id,
            subgroup_ids: c.subgroup_ids.clone(),
        })
        .collect()
}

fn get_vehicles(solution: &Solution) -> Vec<Vehicle> {
    solution
        .instance
        .vehicles
        .iter()
        .map(|v| Vehicle {
            id: v.id,
            budget: v.budget,
            start_node_id: v.start_node_id,
            end_node_id: v.end_node_id,
        })
        .collect()
}

fn get_routes(solution: &Solution) -> Vec<Route> {
    solution
        .routes
        .iter()
        .map(|r| Route {
            vehicle_id: r.vehicle_id,
            path: r.path.clone(),
        })
        .collect()
}

fn get_vehicles_used(solution: &Solution) -> Vec<usize> {
    let mut used_vehicles = HashSet::new();

    for route in &solution.routes {
        if route.path.len() > 2 {
            used_vehicles.insert(route.vehicle_id);
        }
    }

    used_vehicles.into_iter().collect()
}

fn get_clusters_visited(solution: &Solution) -> Vec<usize> {
    let mut visited_clusters = HashSet::new();

    for route in &solution.routes {
        for &node_id in &route.path {
            let node = &solution.instance.nodes[node_id];
            for &sg_id in &node.parent_subgroup_ids {
                for &c_id in &solution.instance.subgroups[sg_id].parent_cluster_ids {
                    visited_clusters.insert(c_id);
                }
            }
        }
    }

    visited_clusters.into_iter().collect()
}

fn get_subgroups_visited(solution: &Solution) -> Vec<usize> {
    let mut visited_subgroups = HashSet::new();

    for route in &solution.routes {
        for &node_id in &route.path {
            let node = &solution.instance.nodes[node_id];
            for &sg_id in &node.parent_subgroup_ids {
                visited_subgroups.insert(sg_id);
            }
        }
    }

    visited_subgroups.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::SolverMode;
    use crate::common::instance::{
        Cluster, Instance, Node, Point3, Subgroup, Vehicle as InstanceVehicle,
    };
    use crate::common::solution::{Route as SolutionRoute, SolutionStatus};
    use std::collections::HashSet;
    use std::fs;
    use std::path::PathBuf;
    use std::time::Duration;

    fn create_test_instance() -> Instance {
        let mut n0 = Node {
            id: 0,
            point: Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            parent_subgroup_ids: HashSet::new(),
        };
        let mut n1 = Node {
            id: 1,
            point: Point3 {
                x: 1.0,
                y: 1.0,
                z: 0.0,
            },
            parent_subgroup_ids: HashSet::new(),
        };
        n0.parent_subgroup_ids.insert(0);
        n1.parent_subgroup_ids.insert(1);

        Instance {
            folder_path: ".".to_string(),
            name: "test_inst".to_string(),
            metric: Metric::Euc2d,
            nodes: vec![n0, n1],
            subgroups: vec![
                Subgroup {
                    id: 0,
                    profit: 10.0,
                    node_ids: vec![0],
                    parent_cluster_ids: HashSet::from([100]),
                },
                Subgroup {
                    id: 1,
                    profit: 20.0,
                    node_ids: vec![1],
                    parent_cluster_ids: HashSet::from([101]),
                },
            ],
            clusters: vec![
                Cluster {
                    id: 100,
                    subgroup_ids: vec![0],
                },
                Cluster {
                    id: 101,
                    subgroup_ids: vec![1],
                },
            ],
            vehicles: vec![InstanceVehicle {
                id: 0,
                budget: 100.0,
                start_node_id: 0,
                end_node_id: 0,
            }],
        }
    }

    #[test]
    fn test_get_mode() {
        let inst = Instance {
            metric: Metric::Euc2d,
            ..Instance::default()
        };
        let sol = Solution {
            instance: &inst,
            duration: Duration::from_secs(1),
            routes: vec![],
            total_cost: 0.0,
            total_score: 0.0,
            status: SolutionStatus::Optimal,
            solver: None,
            best_bound: None,
            gap: None,
            explored_nodes: None,
        };
        assert_eq!(get_mode(&sol), "2d");
    }

    #[test]
    fn test_get_vehicles_used() {
        let inst = create_test_instance();
        let routes = vec![
            SolutionRoute {
                vehicle_id: 0,
                path: vec![0, 0],
                cost: 0.0,
            }, // len <= 2 -> not used
            SolutionRoute {
                vehicle_id: 1,
                path: vec![0, 1, 0],
                cost: 5.0,
            }, // len > 2 -> used
        ];
        let sol = Solution {
            instance: &inst,
            duration: Duration::from_secs(1),
            routes,
            total_cost: 5.0,
            total_score: 20.0,
            status: SolutionStatus::Optimal,
            solver: None,
            best_bound: None,
            gap: None,
            explored_nodes: None,
        };

        let used = get_vehicles_used(&sol);
        assert_eq!(used, vec![1]);
    }

    #[test]
    fn test_get_visited_clusters_and_subgroups() {
        let inst = create_test_instance();
        let routes = vec![SolutionRoute {
            vehicle_id: 0,
            path: vec![0, 1, 0],
            cost: 5.0,
        }];
        let sol = Solution {
            instance: &inst,
            duration: Duration::from_secs(1),
            routes,
            total_cost: 5.0,
            total_score: 30.0,
            status: SolutionStatus::Optimal,
            solver: None,
            best_bound: None,
            gap: None,
            explored_nodes: None,
        };

        let mut sgs = get_subgroups_visited(&sol);
        sgs.sort();
        assert_eq!(sgs, vec![0, 1]);

        let mut cls = get_clusters_visited(&sol);
        cls.sort();
        assert_eq!(cls, vec![100, 101]);
    }

    #[test]
    fn test_export_solution_to_json_success() {
        let temp_dir = tempfile::tempdir().unwrap();
        let folder_result = temp_dir.path().to_str().unwrap().to_string();

        let inst = create_test_instance();
        let sol = Solution {
            instance: &inst,
            duration: Duration::from_secs(2),
            routes: vec![SolutionRoute {
                vehicle_id: 0,
                path: vec![0, 1, 0],
                cost: 10.0,
            }],
            total_cost: 10.0,
            total_score: 30.0,
            status: SolutionStatus::Optimal,
            solver: Some("exact".to_string()),
            best_bound: Some(30.0),
            gap: Some(0.0),
            explored_nodes: Some(5),
        };

        let cli = Cli {
            input: PathBuf::from("test.tcops"),
            mode: SolverMode::Heuristic,
            library: None,
            #[cfg(feature = "lib_good_lp")]
            solver: None,
            max_iterations: 100,
            max_shaking_intensity: 20,
            show: false,
            save: true,
            time_limit: None,
            folder_result: folder_result.clone(),
            custom_result_name: Some("custom_res".to_string()),
            gurobi_params_file: None,
            enable_fractional_cuts: false,
        };

        let output_path = export_solution_to_json(&sol, &cli);
        assert!(output_path.is_some());
        let path_str = output_path.unwrap();
        assert!(path_str.ends_with("custom_res.json"));

        let content = fs::read_to_string(&path_str).unwrap();
        assert!(content.contains("\"instance_name\": \"test_inst\""));
        assert!(content.contains("\"total_score\": 30.0"));
        assert!(content.contains("\"status\": \"Optimal\""));
    }
}
