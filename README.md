# The Team Clustered Orienteering Problem with Subgroups (TCOPS)
*Exact Formulations and Metaheuristics for Multi-Vehicle Routing with Hierarchical Rewards*

[![Rust](https://img.shields.io/badge/rust-2024%20edition-orange.svg)](https://www.rust-lang.org/)
[![Gurobi](https://img.shields.io/badge/solver-Gurobi%2012%2B-red.svg)](https://www.gurobi.com/)

This repository provides the official implementation, instance library, and experimental replication code for the **Team Clustered Orienteering Problem with Subgroups (TCOPS)**.

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Requirements & Installation](#requirements--installation)
  - [Rust Toolchain](#rust-toolchain)
  - [Python Environment](#python-environment)
  - [Mathematical Solvers](#mathematical-solvers)
- [Building the Project](#building-the-project)
- [How to Run the Program](#how-to-run-the-program)
  - [Command-Line Interface (CLI) Arguments](#command-line-interface-cli-arguments)
  - [Execution Examples](#execution-examples)
    - [1. Running the Metaheuristic (VNS)](#1-running-the-metaheuristic-vns)
    - [2. Running the Exact Solver (Gurobi ILP)](#2-running-the-exact-solver-gurobi-ilp)
    - [3. Running with Graphical Visualization](#3-running-with-graphical-visualization)
- [Batch Experiments & Paper Replication](#batch-experiments--paper-replication)
- [Benchmark Instances & File Format](#benchmark-instances--file-format)
- [Solution Output Format](#solution-output-format)
- [Testing](#testing)

---

## Overview

The **Team Clustered Orienteering Problem with Subgroups (TCOPS)** is an extension of the **Clustered Orienteering Problem with Subgroups (COPS)** to cooperative multi-vehicle fleets.



Key components provided in this repository:
- **Exact Solver**: Integer Linear Programming (ILP) formulation solved via **Gurobi** with lazy subtour elimination constraints and symmetry-breaking cuts.
- **Metaheuristic Solver**: High-performance **Variable Neighborhood Search (VNS)** with randomized shaking and multiple local search operators.

---

## Repository Structure

```text
.
├── src/                    # Rust source code
│   ├── main.rs             # TCOPS CLI entrypoint
│   ├── cli/                # Command-line interface definitions (clap)
│   ├── common/             # Instance, solution, and geometric data structures
│   ├── parser/             # Instance parser and validator
│   ├── solvers/            # Exact and Heuristic solvers
│   │   ├── exact/          # ILP formulations (Gurobi)
│   │   └── heuristic/      # Variable Neighborhood Search (VNS) heuristic
│   ├── plotter/            # Visualization
│   ├── printer.rs          # Console report formatter
│   ├── runner.rs           # Solver execution coordinator
│   └── exporter/           # JSON benchmark solution serializer
├── resource/               # Instance library and benchmarks
│   ├── README.md           # Formal .tcops format specification
│   ├── tcops/              # TCOPS benchmark instances
│   ├── clutop/             # Benchmark instances for CluTOP evaluation
│   ├── stop/               # Benchmark instances for STOP evaluation
│   ├── tsp/                # Original TSPLIB 95 instances and best-known solutions
│   └── test/               # Unit and regression test instances
├── script/                 # Python auxiliary scripts
│   ├── experiment.py       # Batch experiment executor benchmarks
│   ├── visualizer.py       # Route plotting and diagram generation
│   ├── tsp2tcops.py        # TSPLIB to TCOPS instance converter
│   └── tsp2clutop_stop.py  # TSPLIB TO CluTOP and STOP instance converter
├── tests/                  # End-to-end and integration tests
├── requirements.txt        # Python package dependencies
├── Cargo.toml              # Rust build configuration and dependencies
├── Makefile                # Build and execution shortcuts
└── gurobi.prm              # Gurobi solver configuration parameters
```

---

## Requirements & Installation

### Rust Toolchain

Install the Rust toolchain (2024 edition compatible):

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup update
```

### Python Environment

A Python 3.10+ environment is recommended for visualization and batch experimentation:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Mathematical Solvers

For exact ILP resolution:
- **Gurobi Optimizer** (v12+): Install the Gurobi package and ensure `GUROBI_HOME` and your license (`gurobi.lic`) are properly configured.
- **SCIP / Good-LP** (Optional): Available via the optional Cargo feature `--features lib_good_lp`. Note: compiling SCIP from source requires `libclang-dev` and `pkg-config`.

---

## Building the Project

Compile the project with maximum release optimizations:

```bash
# Using Cargo
cargo build --release

# Or using Make
make build

# The executable is generated at:
target/release/tcops
```

---

## How to Run the Program

The solver can be executed via Cargo (`cargo run --release -- [ARGS]`), or directly by invoking `./target/release/tcops [ARGS]`.

### Command-Line Interface (CLI) Arguments

| Parameter | Type | Required | Default | Description |
| :--- | :--- | :---: | :---: | :--- |
| `--input <FILE>` | Path | **Yes** | — | Path to the `.tcops` instance file |
| `--mode <MODE>` | Enum | **Yes** | — | Execution mode: `heuristic` or `exact` |
| `--library <LIB>` | Enum | If `exact` | `gurobi` | Mathematical library: `gurobi` (or `good-lp` if enabled) |
| `--time-limit <SEC>` | Integer | No | — | Maximum runtime limit in seconds for exact solvers |
| `--max-iterations <N>` | Integer | No | `100` | Maximum VNS iterations without improvement |
| `--max-shaking-intensity <K>` | Integer | No | `20` | Maximum number of subgroups to displace in VNS shaking |
| `--save` | Flag | No | `false` | Export solution to a JSON file |
| `--folder-result <DIR>` | Path | No | `./result` | Directory where output JSON solutions will be stored |
| `--custom-result-name <NAME>` | String | No | Auto | Custom base name for the result file (without extension) |
| `--show` | Flag | No | `false` | Display interactive graphical plot of routes upon completion |
| `--gurobi-params-file <FILE>` | Path | No | — | Path to custom Gurobi configuration parameter file (e.g. `gurobi.prm`) |

---

### Execution Examples

#### 1. Running the Metaheuristic (VNS)

To solve an instance using the Variable Neighborhood Search metaheuristic:

```bash
# Basic heuristic run via Cargo
cargo run --release -- \
  --input resource/tcops/3/burma14.tcops \
  --mode heuristic

# Or equivalently via Make
make run input=resource/tcops/3/burma14.tcops mode=heuristic

# Heuristic run with customized iterations and result export
cargo run --release -- \
  --input resource/tcops/3/eil76.tcops \
  --mode heuristic \
  --max-iterations 200 \
  --max-shaking-intensity 15 \
  --folder-result ./results/vns \
  --save
```

#### 2. Running the Exact Solver (Gurobi ILP)

To solve an instance to proven mathematical optimality using Gurobi:

```bash
# Exact resolution with a 300-second time limit
cargo run --release -- \
  --input resource/tcops/3/burma14.tcops \
  --mode exact \
  --library gurobi \
  --time-limit 300 \
  --save
```

#### 3. Running with Graphical Visualization

To view the generated routes interactively after solving, pass the `--show` flag:

```bash
cargo run --release -- \
  --input resource/tcops/3/burma14.tcops \
  --mode heuristic \
  --save \
  --show
```

---

## Batch Experiments & Paper Replication

To replicate the computational experiments reported in the paper across exact and heuristic benchmarks:

```bash
# 1. Compile release binary
cargo build --release

# 2. Execute experiment orchestration script
python3 script/experiment.py
```

The script manages:
- 30 independent runs per instance for stochastic metaheuristics.
- Exact solver executions with a 3600s timeout per instance.
- Automatic result logging into `experiment/<problem_type>/<mode>/<vehicles>/`.

---

## Benchmark Instances & File Format

The instance library is hosted under the `resource/` directory. For a complete formal specification of the `.tcops` file format based on the TSPLIB 95 standard, refer to:

**[resource/README.md](resource/README.md)**

Summary of benchmark sets:
- `resource/tcops/`: Benchmark instances partitioned by fleet size $m \in \{1, 2, 3, 4, 5, 7, 9\}$.
- `resource/clutop/`: Benchmark instances for CluTOP evaluation.
- `resource/stop/`: Benchmark instances for STOP evaluation.
- `resource/tsp/`: Original TSPLIB 95 instances with optimal reference values.
- `resource/test/`: Test instances used in the test suite.

---

## Solution Output Format

When `--save` is specified, solutions are serialized to JSON:

```json
{
  "instance_name": "eil76_TCOPS",
  "mode": "2d",
  "solver": "Gurobi",
  "elapsed_time_sec": 45.78,
  "status": "Optimal",
  "total_cost": 508.85,
  "total_score": 1011.0,
  "best_bound": 1011.0,
  "gap": 0.0,
  "explored_nodes": 1612,
  "vehicles_used_ids": [0, 1, 2],
  "clusters_visited_ids": [0, 1, 2, 3],
  "subgroups_visited_ids": [0, 5, 6, 15],
  "routes": [
    { "vehicle_id": 0, "path": [0, 14, 26, 53, 56, 12, 0] },
    { "vehicle_id": 1, "path": [0, 61, 73, 27, 1, 21, 60, 0] },
    { "vehicle_id": 2, "path": [0, 5, 15, 62, 72, 32, 50, 0] }
  ]
}
```

---

## Testing

Run the automated test suite covering parsing, geometric calculations, integrity validation, and solver integration:

```bash
# Run unit tests
cargo test

# Run parser-specific test suite
cargo test parser

# Run end-to-end and integration tests
cargo test --test integration_tests
```

---
