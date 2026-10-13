use clap::Parser;
use tcops::cli::Cli;
use tcops::exporter::io;
use tcops::parser;
use tcops::plotter::plot;
use tcops::printer;
use tcops::solvers::clutop::{self, CluTopOptions};

fn main() {
    let args = Cli::parse();

    println!("Mode: {:?}, Input: {:?}", args.mode, args.input);

    let instance = match parser::load_instance(&args.input) {
        Ok(inst) => inst,
        Err(e) => {
            eprintln!("Fail to load instance {:?}\n{}", args.input, e);
            std::process::exit(1);
        }
    };

    printer::print_instance_info(&instance);

    let options = CluTopOptions {
        mode: args.mode,
        time_limit: args.time_limit,
        max_iterations: args.max_iterations,
        gurobi_params_file: args.gurobi_params_file.clone(),
    };

    let solution = match clutop::solve(&instance, &options) {
        Ok(sol) => sol,
        Err(e) => {
            eprintln!("Fail to solve the instance: {}", e);
            std::process::exit(1);
        }
    };

    printer::print_solution(&solution);

    let path = io::export_solution_to_json(&solution, &args);
    if let Some(path) = path {
        plot::show(&path, args.show, args.save);
    }
}
