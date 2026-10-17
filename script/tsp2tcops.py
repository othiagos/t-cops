import os
import glob
import math
import numpy as np
from sklearn.cluster import KMeans

# Configuration
TSP_DIR = 'resource/tsp'
TCOPS_DIR = 'resource/tcops'
SOLUTIONS_FILE = os.path.join(TSP_DIR, 'solutions')

# Parameters
TARGET_INSTANCE = ""
OVERWRITE = False
THETA = 1.0
SUBGROUPS_PER_CUSTOMER = 0.2
CLUSTERS_PER_SUBGROUP = 0.2

SMALL_INSTANCES = [
    "burma14", "ulysses16", "ulysses22", "att48", "eil51", "berlin52", 
    "st70", "eil76", "pr76", "gr96", "rat99", "kroA100", "rd100", 
    "eil101", "lin105", "pr124", "bier127", "ch130", "pr136", "pr144", 
    "ch150", "kroA150", "pr152", "u159", "rat195", "d198", "kroA200", "gr202", "ts225"
]

LARGE_INSTANCES = [
    "gil262", "a280", "lin318", "rd400", "pcb442", "d493", 
    "att532", "u574", "p654", "d657", "u724", "rat783", "dsj1000",
    "u1060", "pcb1173", "rl1304", "nrw1379", "fl1577", "u1817", "d2103", 
    "pr2392", "pcb3038", "fnl4461", "rl5934", "pla7397", "rl11849", 
    "usa13509", "d18512", "pla33810"
]

INSTANCE_VEHICLES_MAP = {}
for inst in SMALL_INSTANCES:
    INSTANCE_VEHICLES_MAP[inst.lower()] = [1, 2, 3, 4]
for inst in LARGE_INSTANCES:
    INSTANCE_VEHICLES_MAP[inst.lower()] = [5, 7, 9]



def load_solutions(filepath):
    solutions = {}
    if not os.path.exists(filepath):
        print(f"Warning: Solutions file not found at {filepath}")
        return solutions
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line:
                continue
            parts = line.split(':')
            name = parts[0].strip().lower()
            try:
                val_str = parts[1].strip().split()[0]
                solutions[name] = float(val_str)
            except (ValueError, IndexError):
                pass
    return solutions

def parse_tsp_file(filepath):
    nodes = []
    original_comment = "No comment"
    reading_coords = False
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("COMMENT"):
                parts = line.split(":", 1)
                if len(parts) > 1:
                    original_comment = parts[1].strip()
                continue
            if line.startswith("EOF"):
                break
            if line.startswith("NODE_COORD_SECTION"):
                reading_coords = True
                continue
            if reading_coords:
                parts = line.split()
                if len(parts) >= 3:
                    nodes.append((int(parts[0]), float(parts[1]), float(parts[2])))
    return nodes, original_comment

def generate_instances():
    solutions = load_solutions(SOLUTIONS_FILE)
    tsp_files = glob.glob(os.path.join(TSP_DIR, '*.[tT][sS][pP]'))

    if TARGET_INSTANCE:
        tsp_files = [f for f in tsp_files if os.path.basename(f).rsplit('.', 1)[0].lower() == TARGET_INSTANCE.lower()]

    for tsp_file in tsp_files:
        base_name = os.path.basename(tsp_file).rsplit('.', 1)[0]
        search_name = base_name.lower()
        
        if search_name not in INSTANCE_VEHICLES_MAP:
            continue
            
        if search_name not in solutions:
            print(f"Warning: No solution found for {base_name}. Skipping.")
            continue
            
        tsp_opt = solutions[search_name]
        nodes, original_comment = parse_tsp_file(tsp_file)
        
        if len(nodes) < 5:
            continue
            
        depot = nodes[0]
        customers = nodes[1:]
        N = len(customers)
        customer_coords = np.array([[n[1], n[2]] for n in customers])

        num_subgroups = max(2, math.ceil(N * SUBGROUPS_PER_CUSTOMER))
        num_clusters = max(1, math.ceil(num_subgroups * CLUSTERS_PER_SUBGROUP))

        kmeans_sg = KMeans(n_clusters=num_subgroups, n_init=10, random_state=42)
        sg_labels = kmeans_sg.fit_predict(customer_coords)
        sg_centroids = kmeans_sg.cluster_centers_ 

        kmeans_cl = KMeans(n_clusters=num_clusters, n_init=10, random_state=42)
        cl_labels = kmeans_cl.fit_predict(sg_centroids)

        t_max_total = THETA * tsp_opt
        
        common_lines = []
        common_lines.append(f"NAME: {base_name}")
        common_lines.append("TYPE: TCOPS")
        common_lines.append(f"COMMENT: Generated from {base_name}.tsp")
        common_lines.append(f"DIMENSION: {len(nodes)}")
        common_lines.append(f"SUBGROUPS: {num_subgroups + 1}")
        common_lines.append(f"CLUSTERS: {num_clusters + 1}")
        
        node_lines = []
        node_lines.append("EDGE_WEIGHT_TYPE: EUC_2D")
        node_lines.append("NODE_COORD_SECTION: id x y")
        node_lines.append(f"0 {depot[1]:.2f} {depot[2]:.2f}")
        
        subgroup_nodes = {i: [] for i in range(num_subgroups)}
        subgroup_profits = {i: 0.0 for i in range(num_subgroups)}
        
        for idx, (orig_id, x, y) in enumerate(customers):
            new_id = idx + 1
            p_j = 1.0 + ((7141 * new_id + 73) % 100)
            
            node_lines.append(f"{new_id} {x:.2f} {y:.2f}")
            
            sg_id = sg_labels[idx]
            subgroup_nodes[sg_id].append(new_id)
            subgroup_profits[sg_id] += p_j
            
        sg_lines = []
        sg_lines.append("SUBGROUP_SECTION: subgroup_id profit id-vertex-list")
        sg_lines.append("0 0.0 0")
        
        for sg_id in range(num_subgroups):
            nodes_str = " ".join(map(str, subgroup_nodes[sg_id]))
            sg_profit = subgroup_profits[sg_id]
            sg_lines.append(f"{sg_id + 1} {sg_profit:.1f} {nodes_str}")
            
        cl_lines = []
        cl_lines.append("CLUSTER_SECTION: cluster_id id-subgroup-list")
        cl_lines.append("0 0")
        cluster_sgs = {i: [] for i in range(num_clusters)}
        for sg_index, c_label in enumerate(cl_labels):
            cluster_sgs[c_label].append(sg_index + 1)
            
        for c_id in range(num_clusters):
            sgs_str = " ".join(map(str, cluster_sgs[c_id]))
            cl_lines.append(f"{c_id + 1} {sgs_str}")

        required_vehicles = INSTANCE_VEHICLES_MAP[search_name]
        
        for v in required_vehicles:
            output_dir = os.path.join(TCOPS_DIR, str(v))
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, f"{base_name}.tcops")
            
            if os.path.exists(output_path) and not OVERWRITE:
                continue
                
            budget_per_vehicle = math.ceil(t_max_total / v)
            
            v_lines = []
            v_lines.append(f"VEHICLES: {v}")
            
            v_section_lines = []
            v_section_lines.append("VEHICLES_SECTION: id tmax start_node_id end_node_id")
            for v_id in range(v):
                v_section_lines.append(f"{v_id} {budget_per_vehicle:.1f} 0 0")
            
            final_content = (
                common_lines + 
                v_lines + 
                node_lines + 
                sg_lines + 
                cl_lines + 
                v_section_lines
            )
            
            with open(output_path, 'w') as f:
                f.write("\n".join(final_content) + "\n")
                
        print(f"[OK] {base_name} generated for vehicles: {required_vehicles}")

if __name__ == "__main__":
    generate_instances()