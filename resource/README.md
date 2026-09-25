# Input File Format (`.tcops`)
*Based on the TSPLIB 95 specification format (Gerhard Reinelt)*

Each file consists of a **specification part** and a **data part**. The specification part contains metadata and configuration information on the file format and its contents. The data part contains the explicit instance data.

---

## 1. The file format

### 1.1 The specification part

All entries in this section are of the form `<keyword> : <value>`, where `<keyword>` denotes an alphanumerical keyword and `<value>` denotes alphanumerical or numerical data. The terms `<string>`, `<integer>`, and `<real>` denote character strings, non-negative integers, and floating-point real numbers, respectively. Empty lines or lines starting with `#` are treated as comments and ignored.

Below is the list of all available keywords:

#### 1.1.1 `NAME : <string>`
Identifies the name of the instance.

#### 1.1.2 `TYPE : <string>`
Specifies the type of problem represented by the instance. Possible types are:
- `TCOPS`: Team Clustered Orienteering Problem with Subgroups
- `CluTOP`: Clustered Team Orienteering Problem
- `STOP`: Set Team Orienteering Problem
- `COPS`: Clustered Orienteering Problem with Subgroups (single vehicle)

#### 1.1.3 `COMMENT : <string>`
Additional comments (such as instance origin, contributor, generation parameters, or reference TSP file).

#### 1.1.4 `DIMENSION : <integer>`
Specifies the total number of nodes in the instance (including depots and customers), denoted by $n = |N|$. Nodes are indexed from $0$ to $n - 1$.

#### 1.1.5 `SUBGROUPS : <integer>`
Specifies the total number of subgroups in the instance, denoted by $s = |S|$. Subgroups are indexed from $0$ to $s - 1$.

#### 1.1.6 `CLUSTERS : <integer>`
Specifies the total number of clusters in the instance, denoted by $c = |C|$. Clusters are indexed from $0$ to $c - 1$.

#### 1.1.7 `VEHICLES : <integer>`
Specifies the number of vehicles available in the fleet, denoted by $v = |V|$. Vehicles are indexed from $0$ to $v - 1$.

#### 1.1.8 `EDGE_WEIGHT_TYPE : <string>`
Specifies how arc traversal costs $c_{ij}$ (distances between nodes) are computed. The possible values are:
- `EUC_2D`: Euclidean distances in 2-D
- `EUC_3D`: Euclidean distances in 3-D
- `MAN_2D`: Manhattan distances in 2-D
- `MAN_3D`: Manhattan distances in 3-D

---

### 1.2 The data part

Explicit problem data are provided in corresponding sections following the specification part. Each data section begins with the corresponding keyword. The number of entries in each section is determined exactly by the values declared in the specification part.

#### 1.2.1 `NODE_COORD_SECTION`
Node coordinates are specified in this section. The section contains exactly `DIMENSION` lines, each of the form:

```text
<integer> <real> <real>
```
if `EDGE_WEIGHT_TYPE` is in 2-D (`<id> <x> <y>`), or:
```text
<integer> <real> <real> <real>
```
if `EDGE_WEIGHT_TYPE` is in 3-D (`<id> <x> <y> <z>`).

- The first number (`<integer>`) specifies the unique node identifier, starting at $0$ and strictly sequential ($0, 1, \dots, \text{DIMENSION}-1$).
- The following real numbers specify the associated Cartesian coordinates.
- By convention, node $0$ represents the primary depot.

#### 1.2.2 `SUBGROUP_SECTION`
Subgroups of nodes and their associated profits are defined in this section. The section contains exactly `SUBGROUPS` lines, each of the form:

```text
<integer> <real> <integer> ... <integer>
```
*(i.e., `<subgroup_id> <profit> <node_1> <node_2> ... <node_k>`)*

- The first integer specifies the subgroup identifier, starting at $0$ and strictly sequential ($0, 1, \dots, s-1$).
- The real number specifies the profit (or reward $P_u$) awarded for visiting all nodes in the subgroup.
- The subsequent integers list the node identifiers belonging to this subgroup.
- By convention, subgroup $0$ is the depot subgroup with profit `0.0`, containing only the depot node (e.g., `0 0.0 0`).
- A node may belong to more than one subgroup (overlapping subgroups are permitted).

#### 1.2.3 `CLUSTER_SECTION`
Clusters grouping the subgroups are defined in this section. The section contains exactly `CLUSTERS` lines, each of the form:

```text
<integer> <integer> ... <integer>
```
*(i.e., `<cluster_id> <subgroup_1> <subgroup_2> ... <subgroup_m>`)*

- The first integer specifies the cluster identifier, starting at $0$ and strictly sequential ($0, 1, \dots, c-1$).
- The subsequent integers list the subgroup identifiers belonging to this cluster ($S_g$).
- By convention, cluster $0$ is the depot cluster grouping subgroup $0$ (e.g., `0 0`).
- A subgroup may belong to more than one cluster (overlapping clusters are permitted).

#### 1.2.4 `VEHICLES_SECTION`
Fleet vehicle parameters and operational constraints are defined in this section. The section contains exactly `VEHICLES` lines, each of the form:

```text
<integer> <real> <integer> <integer>
```
*(i.e., `<vehicle_id> <tmax> <start_node_id> <end_node_id>`)*

- The first integer specifies the vehicle identifier $k$, starting at $0$ and strictly sequential ($0, 1, \dots, v-1$).
- The real number specifies the maximum travel time or distance budget allowed for the vehicle ($T_{max} \equiv B_k$).
- The third and fourth integers specify the starting node identifier ($v_o^k$) and destination node identifier ($v_d^k$), respectively. If both identifiers are equal, the tour is closed (round trip); if they differ, the tour is open between distinct depots.

---

## 2. The distance functions

For the different choices of `EDGE_WEIGHT_TYPE`, the traversal cost $c_{ij} = d_{ij}$ between two nodes $i$ and $j$ is computed from their coordinates using double-precision floating-point arithmetic.

### 2.1 Euclidean distance

For 2-dimensional coordinates $(x_i, y_i)$ and $(x_j, y_j)$ (`EUC_2D`):

$$d_{ij} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2}$$

For 3-dimensional coordinates $(x_i, y_i, z_i)$ and $(x_j, y_j, z_j)$ (`EUC_3D`):

$$d_{ij} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2 + (z_i - z_j)^2}$$

### 2.2 Manhattan distance

For 2-dimensional coordinates $(x_i, y_i)$ and $(x_j, y_j)$ (`MAN_2D`):

$$d_{ij} = |x_i - x_j| + |y_i - y_j|$$

For 3-dimensional coordinates $(x_i, y_i, z_i)$ and $(x_j, y_j, z_j)$ (`MAN_3D`):

$$d_{ij} = |x_i - x_j| + |y_i - y_j| + |z_i - z_j|$$

---

## 3. Structural integrity rules

To ensure consistency of instance files, the following conditions must be satisfied:

1. **Zero-based sequential indexing**:
   All identifiers for nodes, subgroups, clusters, and vehicles must start at $0$ and be strictly sequential ($0, 1, 2, \dots, K-1$).
2. **Cardinality consistency**:
   The number of entries present in each data section must match exactly the dimensions declared in the specification part (`DIMENSION`, `SUBGROUPS`, `CLUSTERS`, `VEHICLES`).
3. **Referential integrity**:
   - Every node referenced in a subgroup must exist in the node coordinates section.
   - Every subgroup referenced in a cluster must exist in the subgroups section.
   - Vehicle starting and ending nodes must exist in the node coordinates section.
4. **Hierarchy without orphan elements**:
   - Every node must belong to at least one subgroup.
   - Every subgroup must belong to at least one cluster.
