import os
import subprocess
import multiprocessing
from concurrent.futures import ThreadPoolExecutor, as_completed

# Configuration
EXPERIMENT_DIR = "experiment"

EXECUTIONS_EXACT = 1
EXECUTIONS_HEURISTIC = 30
TIME_LIMIT_EXACT = 3600

VEHICLES_EXACT = [1, 2, 3, 4]
VEHICLES_HEURISTIC_SMALL = [3]
VEHICLES_HEURISTIC_LARGE = [5, 7, 9]

VEHICLES_CLUTOP = [2, 3, 4]
VEHICLES_STOP = [2, 3, 4]

VEHICLES_TCOPS_CLUTOP = [2, 3, 4]
VEHICLES_TCOPS_STOP = [2, 3, 4]

RESULT_EXTENSION = ".json" 
EXECUTABLE = "target/release/tcops"
EXECUTABLE_CLUTOP = "target/release/clutop"
EXECUTABLE_STOP = "target/release/stop"

# Instances
EXACT_INSTANCES = [
    "burma14", "ulysses16", "ulysses22", "att48", "eil51", "berlin52", 
    "st70", "eil76", "pr76", "gr96", "rat99", "kroA100", "rd100", 
    "eil101", "lin105", "pr124", "bier127", "ch130", "pr136", "pr144", 
    "ch150", "kroA150", "pr152", "u159", "rat195", "d198", "kroA200", "gr202", "ts225"
]

HEURISTIC_LARGE_INSTANCES = [
    "gil262", "a280", "lin318", "rd400", "pcb442", "d493", 
    "att532", "u574", "p654", "d657", "u724", "rat783", "dsj1000",
    "u1060", "pcb1173", "rl1304", "nrw1379", "fl1577", "u1817", "d2103", 
    "pr2392", "pcb3038", "fnl4461", "rl5934", "pla7397", "rl11849", 
    "usa13509", "d18512", "pla33810"
]

def run_experiment(problem_type, mode, instance, vehicles, iteration, executable=None, folder_name=None):
    input_file = f"resource/{problem_type}/{vehicles}/{instance}.tcops"
    output_folder_name = folder_name if folder_name is not None else problem_type
    folder_result = f"{EXPERIMENT_DIR}/{output_folder_name}/{mode}/{vehicles}"
    custom_result_name = f"{instance}_{iteration}"
    
    expected_output_file = os.path.join(folder_result, f"{custom_result_name}{RESULT_EXTENSION}")

    if os.path.exists(expected_output_file):
        print(f"[SKIP] {output_folder_name.upper()} | {mode.upper()} | {instance} | V={vehicles} | Iter={iteration} -> Already completed.")
        return

    if not os.path.exists(input_file):
        print(f"[WARNING] Input file not found: {input_file}. Skipping...")
        return

    os.makedirs(folder_result, exist_ok=True)

    if executable is None:
        if problem_type == "clutop":
            executable = EXECUTABLE_CLUTOP
        elif problem_type == "stop":
            executable = EXECUTABLE_STOP
        else:
            executable = EXECUTABLE

    cmd = [
        executable,
        "--input", input_file,
        "--mode", mode,
        "--folder-result", folder_result,
        "--custom-result-name", custom_result_name,
        "--save"
    ]

    if mode == "exact":
        cmd.extend([
            "--library", "gurobi",
            "--time-limit", str(TIME_LIMIT_EXACT)
        ])

    print(f"[EXEC] Running: {output_folder_name.upper()} | {mode.upper()} | {instance} | V={vehicles} | Iter={iteration}")
    
    try:
        subprocess.run(cmd, check=True, text=True)
    except subprocess.CalledProcessError as e:
        print(f"\n[SOLVER ERROR] Failed on instance {instance} (V={vehicles}). Code: {e.returncode}.\n")
    except Exception as e:
        print(f"\n[CRITICAL ERROR] Unexpected failure when trying to run {instance}: {e}\n")


def run_experiment_wrapper(args):
    run_experiment(*args)


def main():
    print("Starting batch of TCOPS/CluTOP/STOP experiments...\n")

    for exe in [EXECUTABLE, EXECUTABLE_CLUTOP, EXECUTABLE_STOP]:
        if not os.path.exists(exe):
            print(f"[FATAL ERROR] The executable '{exe}' was not found.")
            return
    
    for instance in EXACT_INSTANCES:
        for v in VEHICLES_EXACT:
            for i in range(1, EXECUTIONS_EXACT + 1):
                # TCOPS
                run_experiment("tcops", "exact", instance, v, i, executable=EXECUTABLE, folder_name="tcops")

                # TCOPS running CluTOP instances
                if v in VEHICLES_TCOPS_CLUTOP:
                    run_experiment("clutop", "exact", instance, v, i, executable=EXECUTABLE, folder_name="tcops_clutop")

                # TCOPS running STOP instances
                if v in VEHICLES_TCOPS_STOP:
                    run_experiment("stop", "exact", instance, v, i, executable=EXECUTABLE, folder_name="tcops_stop")
                
                # CluTOP
                if v in VEHICLES_CLUTOP:
                    run_experiment("clutop", "exact", instance, v, i, executable=EXECUTABLE_CLUTOP, folder_name="clutop")
                    
                # STOP
                if v in VEHICLES_STOP:
                    run_experiment("stop", "exact", instance, v, i, executable=EXECUTABLE_STOP, folder_name="stop")
    
    heuristic_tasks = []

    for instance in EXACT_INSTANCES:
        for v in VEHICLES_HEURISTIC_SMALL:
            for j in range(1, EXECUTIONS_HEURISTIC + 1):
                heuristic_tasks.append(("tcops", "heuristic", instance, v, j))

    for instance in HEURISTIC_LARGE_INSTANCES:
        for v in VEHICLES_HEURISTIC_LARGE:
            for i in range(1, EXECUTIONS_HEURISTIC + 1):
                heuristic_tasks.append(("tcops", "heuristic", instance, v, i))

    total_tasks = len(heuristic_tasks)
    if total_tasks > 0:
        max_threads = multiprocessing.cpu_count()
        print(f"Found {total_tasks} pending heuristic executions.")
        print(f"Allocating {max_threads} threads for continuous processing...\n")

        with ThreadPoolExecutor(max_workers=max_threads) as executor:
            futures = {executor.submit(run_experiment_wrapper, task): task for task in heuristic_tasks}
            
            for future in as_completed(futures):
                try:
                    future.result()
                except Exception as e:
                    task = futures[future]
                    print(f"[THREAD FAILURE] Task {task} failed with error: {e}")
    else:
        print("-> No heuristic tasks configured to run.")

    print("\n[END] All experiments completed successfully!")

if __name__ == "__main__":
    main()