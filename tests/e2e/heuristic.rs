use std::fs;
use std::path::PathBuf;
use tcops::{
    cli::{Cli, SolverMode},
    runner::run,
};
use tempfile::tempdir;

fn create_e2e_cli(
    input_path: &str,
    folder_result: &str,
    custom_name: Option<String>,
) -> Cli {
    Cli {
        input: PathBuf::from(input_path),
        mode: SolverMode::Heuristic,
        library: None,
        #[cfg(feature = "lib_good_lp")]
        solver: None,
        max_iterations: 50,
        max_shaking_intensity: 10,
        show: false,
        save: false,
        time_limit: None,
        folder_result: folder_result.to_string(),
        custom_result_name: custom_name,
        gurobi_params_file: None,
        enable_fractional_cuts: false,
    }
}

/// E2E test for Heuristic solver on burma14.tcops
#[test]
fn test_e2e_heuristic_burma14() {
    let temp_dir = tempdir().expect("Failed to create temp dir");
    let result_folder = temp_dir.path().to_str().unwrap();

    let cli = create_e2e_cli(
        "resource/tcops/3/burma14.tcops",
        result_folder,
        Some("burma14_heur_e2e".to_string()),
    );

    let res = run(cli);
    assert!(res.is_ok(), "runner::run for Heuristic should succeed");

    let expected_json_path = temp_dir.path().join("burma14_heur_e2e.json");
    assert!(expected_json_path.exists(), "JSON result file should be created");

    let content = fs::read_to_string(&expected_json_path).expect("Failed to read output JSON");
    let json_val: serde_json::Value = serde_json::from_str(&content).expect("Invalid JSON");

    assert_eq!(json_val["instance_name"], "burma14");
    assert_eq!(json_val["status"], "Feasible");
    assert!(json_val["total_score"].as_f64().unwrap() > 0.0);
    assert!(json_val["total_cost"].as_f64().unwrap() > 0.0);
    assert!(!json_val["routes"].as_array().unwrap().is_empty());
}

/// E2E test for Heuristic solver on ulysses16.tcops
#[test]
fn test_e2e_heuristic_ulysses16() {
    let temp_dir = tempdir().expect("Failed to create temp dir");
    let result_folder = temp_dir.path().to_str().unwrap();

    let cli = create_e2e_cli(
        "resource/tcops/3/ulysses16.tcops",
        result_folder,
        Some("ulysses16_heur_e2e".to_string()),
    );

    let res = run(cli);
    assert!(res.is_ok(), "Heuristic solve for ulysses16 should succeed");

    let json_path = temp_dir.path().join("ulysses16_heur_e2e.json");
    assert!(json_path.exists());
}
