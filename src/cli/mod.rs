use clap::{Parser, ValueEnum, ValueHint};
use std::path::PathBuf;

#[derive(Parser, Debug, Clone)]
#[command(
    author,
    version,
    about = "TCOPS Solver Engine",
    long_about = "Resolution engine for the Team Clustered Orienteering Problem with Subgroups."
)]
pub struct Cli {
    /// Path to the instance file (.tcops)
    #[arg(long, value_name = "FILE", value_hint = ValueHint::FilePath)]
    pub input: PathBuf,

    /// Algorithm execution mode (exact or heuristic)
    #[arg(long, value_enum)]
    pub mode: SolverMode,

    /// Mathematical library to use for the exact solver (Required if mode=exact)
    #[arg(long, value_enum, required_if_eq("mode", "exact"))]
    pub library: Option<LibraryType>,

    /// Mathematical solver type (Required if library=good-lp)
    #[cfg(feature = "lib_good_lp")]
    #[arg(long, value_enum, required_if_eq("library", "good-lp"))]
    pub solver: Option<ExactSolverType>,

    /// Maximum iterations without improvement for the VNS (Only for mode=heuristic)
    #[arg(long, default_value_t = 100)]
    pub max_iterations: usize,

    /// Maximum number of subgroups to remove during the shaking phase
    #[arg(long, default_value_t = 20)]
    pub max_shaking_intensity: usize,

    /// Displays the detailed solution in the terminal at the end of the execution
    #[arg(long, default_value_t = false)]
    pub show: bool,

    /// Saves the solution result to an output file
    #[arg(long, default_value_t = false)]
    pub save: bool,

    /// Time limit for the exact solver gurobi in seconds (Only for mode=exact)
    #[arg(long)]
    pub time_limit: Option<u64>,

    /// Folder to save the results
    #[arg(long, default_value = "./result", value_hint = ValueHint::DirPath)]
    pub folder_result: String,

    /// Custom name for the result file (without extension)
    #[arg(long)]
    pub custom_result_name: Option<String>,

    /// Gurobi parameters file (Only for mode=exact and library=gurobi)
    #[arg(long, value_name = "FILE", value_hint = ValueHint::FilePath)]
    pub gurobi_params_file: Option<String>,

    /// Enable fractional cuts separation at MIPNode callback (Only for mode=exact and library=gurobi)
    #[arg(long, alias = "fractional-cuts", default_value_t = false)]
    pub enable_fractional_cuts: bool,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
pub enum SolverMode {
    Exact,
    Heuristic,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
pub enum LibraryType {
    #[cfg(feature = "lib_good_lp")]
    GoodLp,
    Gurobi,
}

#[cfg(feature = "lib_good_lp")]
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
pub enum ExactSolverType {
    Gurobi,
    Scip,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_heuristic_mode_minimal() {
        let args = vec!["tcops", "--input", "data/tcops/att48.tcops", "--mode", "heuristic"];
        let cli = Cli::try_parse_from(args).expect("Failed to parse minimal heuristic CLI");

        assert_eq!(cli.input, PathBuf::from("data/tcops/att48.tcops"));
        assert_eq!(cli.mode, SolverMode::Heuristic);
        assert_eq!(cli.library, None);
        assert_eq!(cli.max_iterations, 100);
        assert_eq!(cli.max_shaking_intensity, 20);
        assert!(!cli.show);
        assert!(!cli.save);
        assert_eq!(cli.folder_result, "./result");
        assert_eq!(cli.custom_result_name, None);
        assert_eq!(cli.gurobi_params_file, None);
    }

    #[test]
    fn test_cli_heuristic_mode_full() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "heuristic",
            "--max-iterations", "500",
            "--max-shaking-intensity", "50",
            "--show",
            "--save",
            "--folder-result", "./my_results",
            "--custom-result-name", "heuristic_run_1",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse full heuristic CLI");

        assert_eq!(cli.input, PathBuf::from("data/tcops/att48.tcops"));
        assert_eq!(cli.mode, SolverMode::Heuristic);
        assert_eq!(cli.max_iterations, 500);
        assert_eq!(cli.max_shaking_intensity, 50);
        assert!(cli.show);
        assert!(cli.save);
        assert_eq!(cli.folder_result, "./my_results");
        assert_eq!(cli.custom_result_name, Some("heuristic_run_1".to_string()));
    }

    #[test]
    fn test_cli_exact_mode_gurobi_library_minimal() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "gurobi",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse exact gurobi CLI");

        assert_eq!(cli.input, PathBuf::from("data/tcops/att48.tcops"));
        assert_eq!(cli.mode, SolverMode::Exact);
        assert_eq!(cli.library, Some(LibraryType::Gurobi));
        assert_eq!(cli.time_limit, None);
    }

    #[test]
    fn test_cli_exact_mode_gurobi_library_full() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "gurobi",
            "--time-limit", "600",
            "--gurobi-params-file", "gurobi.prm",
            "--show",
            "--save",
            "--folder-result", "./exact_out",
            "--custom-result-name", "exact_gurobi_test",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse full exact gurobi CLI");

        assert_eq!(cli.mode, SolverMode::Exact);
        assert_eq!(cli.library, Some(LibraryType::Gurobi));
        assert_eq!(cli.time_limit, Some(600));
        assert_eq!(cli.gurobi_params_file, Some("gurobi.prm".to_string()));
        assert!(cli.show);
        assert!(cli.save);
        assert_eq!(cli.folder_result, "./exact_out");
        assert_eq!(cli.custom_result_name, Some("exact_gurobi_test".to_string()));
    }


    #[cfg(feature = "lib_good_lp")]
    #[test]
    fn test_cli_exact_mode_good_lp_gurobi_solver() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "good-lp",
            "--solver", "gurobi",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse good-lp gurobi CLI");

        assert_eq!(cli.mode, SolverMode::Exact);
        assert_eq!(cli.library, Some(LibraryType::GoodLp));
        assert_eq!(cli.solver, Some(ExactSolverType::Gurobi));
    }

    #[cfg(feature = "lib_good_lp")]
    #[test]
    fn test_cli_exact_mode_good_lp_scip_solver() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "good-lp",
            "--solver", "scip",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse good-lp scip CLI");

        assert_eq!(cli.mode, SolverMode::Exact);
        assert_eq!(cli.library, Some(LibraryType::GoodLp));
        assert_eq!(cli.solver, Some(ExactSolverType::Scip));
    }

    #[test]
    fn test_cli_invalid_missing_input() {
        let args = vec!["tcops", "--mode", "heuristic"];
        assert!(Cli::try_parse_from(args).is_err());
    }

    #[test]
    fn test_cli_invalid_missing_mode() {
        let args = vec!["tcops", "--input", "data/tcops/att48.tcops"];
        assert!(Cli::try_parse_from(args).is_err());
    }

    #[test]
    fn test_cli_invalid_exact_without_library() {
        let args = vec!["tcops", "--input", "data/tcops/att48.tcops", "--mode", "exact"];
        assert!(Cli::try_parse_from(args).is_err());
    }

    #[test]
    fn test_cli_invalid_mode_value() {
        let args = vec!["tcops", "--input", "data/tcops/att48.tcops", "--mode", "invalid_mode"];
        assert!(Cli::try_parse_from(args).is_err());
    }

    #[test]
    fn test_cli_invalid_library_value() {
        let args = vec!["tcops", "--input", "data/tcops/att48.tcops", "--mode", "exact", "--library", "invalid_lib"];
        assert!(Cli::try_parse_from(args).is_err());
    }

    #[test]
    fn test_cli_fractional_cuts_flag_default_is_false() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "gurobi",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse minimal exact CLI");
        assert!(!cli.enable_fractional_cuts);
    }

    #[test]
    fn test_cli_fractional_cuts_flag_parsing() {
        let args = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "gurobi",
            "--enable-fractional-cuts",
        ];
        let cli = Cli::try_parse_from(args).expect("Failed to parse exact CLI with --enable-fractional-cuts");
        assert!(cli.enable_fractional_cuts);

        let args_alias = vec![
            "tcops",
            "--input", "data/tcops/att48.tcops",
            "--mode", "exact",
            "--library", "gurobi",
            "--fractional-cuts",
        ];
        let cli_alias = Cli::try_parse_from(args_alias).expect("Failed to parse exact CLI with --fractional-cuts");
        assert!(cli_alias.enable_fractional_cuts);
    }
}

