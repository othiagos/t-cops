import os
import glob
import math
import numpy as np
from sklearn.cluster import KMeans

# Configuration
TSP_DIR = 'resource/tsp'
OUTPUT_DIR_CLUTOP = 'resource/clutop/3'
OUTPUT_DIR_STOP = 'resource/stop/3'
SOLUTIONS_FILE = os.path.join(TSP_DIR, 'solutions')

# Parameters
OVERWRITE = False
THETA = 1.0
VEHICLES = 3
CLUSTERS_FACTOR_CLUTOP = 0.2
CLUSTERS_FACTOR_STOP = 0.2

INSTANCES = [
    "burma14", "ulysses16", "ulysses22", "att48", "eil51", "berlin52", 
    "st70", "eil76", "pr76", "gr96", "rat99", "kroA100", "rd100", 
    "eil101", "lin105", "pr124", "bier127", "ch130", "pr136", "pr144", 
    "ch150", "kroA150", "pr152", "u159", "rat195", "d198", "kroA200", "gr202", "ts225"
]

def load_solutions(filepath):
    solutions = {}
    if not os.path.exists(filepath):
        print(f"Warning: Solutions file not found at {filepath}")
        return solutions
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line: continue
            parts = line.split(':')
            try:
                solutions[parts[0].strip().lower()] = float(parts[1].strip().split()[0])
            except ValueError:
                pass
    return solutions

def parse_tsp(filepath):
    nodes = []
    reading = False
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("EOF"): break
            if line.startswith("NODE_COORD_SECTION"):
                reading = True
                continue
            if reading:
                parts = line.split()
                if len(parts) >= 3:
                    nodes.append((int(parts[0]), float(parts[1]), float(parts[2])))
    return nodes

def generate_file(output_path, base_name, problem_type, nodes, depot, sg_mapping, cl_mapping, budget):
    if os.path.exists(output_path) and not OVERWRITE:
        return

    num_subgroups = len(sg_mapping)
    num_clusters = len(cl_mapping)
    
    lines = []
    lines.append(f"NAME: {base_name}_{problem_type}")
    lines.append(f"TYPE: {problem_type}")
    lines.append(f"COMMENT: Instancia gerada com fator especifico para {problem_type}")
    lines.append(f"DIMENSION: {len(nodes) + 1}")
    lines.append(f"SUBGROUPS: {num_subgroups + 1}")
    lines.append(f"CLUSTERS: {num_clusters + 1}")
    lines.append(f"VEHICLES: {VEHICLES}")
    lines.append("EDGE_WEIGHT_TYPE: EUC_2D")
    
    lines.append("NODE_COORD_SECTION: id x y")
    lines.append(f"0 {depot[1]:.2f} {depot[2]:.2f}")
    
    node_profits = {}
    
    for idx, (orig_id, x, y) in enumerate(nodes):
        new_id = idx + 1
        p_j = 1.0 + ((7141 * new_id + 73) % 100) 
        node_profits[new_id] = p_j
        lines.append(f"{new_id} {x:.2f} {y:.2f}")
        
    lines.append("SUBGROUP_SECTION: subgroup_id profit id-vertex-list")
    lines.append("0 0.0 0")
    
    for sg_id, node_list in sg_mapping.items():
        nodes_str = " ".join(map(str, node_list))
        sg_profit = sum(node_profits[nid] for nid in node_list)
        lines.append(f"{sg_id} {sg_profit:.1f} {nodes_str}")
        
    lines.append("CLUSTER_SECTION: cluster_id id-subgroup-list")
    lines.append("0 0")
    for cl_id, sg_list in cl_mapping.items():
        sgs_str = " ".join(map(str, sg_list))
        lines.append(f"{cl_id} {sgs_str}")
        
    lines.append("VEHICLES_SECTION: id tmax start_node_id end_node_id")
    for v_id in range(VEHICLES):
        lines.append(f"{v_id} {budget:.1f} 0 0")
        
    with open(output_path, 'w') as f:
        f.write("\n".join(lines) + "\n")

def main():
    os.makedirs(OUTPUT_DIR_CLUTOP, exist_ok=True)
    os.makedirs(OUTPUT_DIR_STOP, exist_ok=True)
    
    solutions = load_solutions(SOLUTIONS_FILE)
    
    for inst_name in INSTANCES:
        tsp_file = os.path.join(TSP_DIR, f"{inst_name}.tsp")
        if not os.path.exists(tsp_file):
            tsp_file = os.path.join(TSP_DIR, f"{inst_name}.TSP")
            if not os.path.exists(tsp_file):
                print(f"Warning: TSP file not found for {inst_name}. Skipping.")
                continue
                
        if inst_name.lower() not in solutions:
            print(f"Warning: No solution found for {inst_name}. Skipping.")
            continue
            
        nodes = parse_tsp(tsp_file)
        if len(nodes) < 5: continue
            
        depot = nodes[0]
        customers = nodes[1:]
        N = len(customers)
        coords = np.array([[n[1], n[2]] for n in customers])
        
        tsp_opt = solutions[inst_name.lower()]
        budget_per_vehicle = math.ceil((THETA * tsp_opt) / VEHICLES)
        
        num_clusters_clutop = max(1, math.ceil(N * CLUSTERS_FACTOR_CLUTOP))
        kmeans_clutop = KMeans(n_clusters=num_clusters_clutop, n_init=10, random_state=42)
        labels_clutop = kmeans_clutop.fit_predict(coords)
        
        clutop_sg_map = {c_id: [] for c_id in range(1, num_clusters_clutop + 1)}
        clutop_cl_map = {c_id: [c_id] for c_id in range(1, num_clusters_clutop + 1)}
        
        for idx, cluster_idx in enumerate(labels_clutop):
            node_id = idx + 1
            cluster_id = cluster_idx + 1
            clutop_sg_map[cluster_id].append(node_id) 
            
        generate_file(
            os.path.join(OUTPUT_DIR_CLUTOP, f"{inst_name}.tcops"),
            inst_name, "CluTOP", customers, depot, clutop_sg_map, clutop_cl_map, budget_per_vehicle
        )
        
        num_clusters_stop = max(1, math.ceil(N * CLUSTERS_FACTOR_STOP))
        kmeans_stop = KMeans(n_clusters=num_clusters_stop, n_init=10, random_state=42)
        labels_stop = kmeans_stop.fit_predict(coords)
        
        stop_sg_map = {}
        stop_cl_map = {c_id: [] for c_id in range(1, num_clusters_stop + 1)}
        
        for idx, cluster_idx in enumerate(labels_stop):
            node_id = idx + 1
            cluster_id = cluster_idx + 1
            fake_sg_id = node_id
            
            stop_sg_map[fake_sg_id] = [node_id] 
            stop_cl_map[cluster_id].append(fake_sg_id) 
            
        generate_file(
            os.path.join(OUTPUT_DIR_STOP, f"{inst_name}.tcops"),
            inst_name, "STOP", customers, depot, stop_sg_map, stop_cl_map, budget_per_vehicle
        )
        
        print(f"[OK] {inst_name} generated for vehicles: [{VEHICLES}]")

if __name__ == "__main__":
    main()