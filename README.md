# Vehicle Routing Problem (VRP)

A **Mixed Integer Linear Programming (MILP)** model in **Python** for the **Capacitated Vehicle Routing Problem**, built with the **[Pyomo](http://www.pyomo.org/)** optimization framework and solved via the **IBM ILOG CPLEX** solver.

## Overview

The Vehicle Routing Problem (VRP) is a fundamental combinatorial optimization problem in logistics and Operations Research. A fleet of trucks must deliver goods from a central depot to a set of customers, each with a known demand. Every truck has a fixed capacity that cannot be exceeded, every customer must be served by exactly one truck, and the objective is to **minimize the total travel distance** across all routes.

Unlike the TSP, the VRP implicitly determines the **number and composition of routes** needed to serve all customers within the capacity constraint. Subtour elimination and vehicle assignment are handled jointly through a **commodity flow formulation**, which uses a continuous flow variable $f_{ij}$ representing the number of goods units carried on each arc.

## Repository Contents

| File | Description |
|---|---|
| `Vehicle_Routing_Problem_VRP.py` | Python script implementing and solving the VRP via Pyomo and CPLEX |
| `Vehicle_Routing_Problem_VRP_Problem_CLSP_Problem_Results.txt` | Solver output: execution time, status, optimal cost, active arcs, flow values, and flow balance verification for every node |
| `VRP_Math_Model.pdf` | Mathematical formulation of the problem |

## Mathematical Formulation

### Parameters

- $n$ = number of nodes (node $1$ is the depot; nodes $2, \dots, n$ are customers)
- $d_{ij}$ = distance from node $i$ to node $j$; $\forall\, i, j = 1, \dots, n$
- $D_i$ = demand of customer $i$; $\forall\, i = 2, \dots, n$
- $C$ = capacity of each truck

### Variables

- $f_{ij}$ = number of goods units carried by a truck going from node $i$ to node $j$; $\forall\, i, j = 1, \dots, n$
- $x_{ij}$ = binary routing variable:

$$
x_{ij} = \begin{cases} 1 & \text{if a truck goes from node } i \text{ to node } j \\ 0 & \text{otherwise} \end{cases}
$$

### Objective Function

**(1)** — Minimize total travel distance across all routes

$$
\displaystyle \min \sum_{i=1}^{n} \sum_{j=1}^{n} d_{ij} \cdot x_{ij}
$$

### Constraints

**(2)** — Each customer is entered exactly once

$$
\displaystyle \sum_{j=1}^{n} x_{ij} = 1 \qquad \forall\, i = 2, \dots, n
$$

**(3)** — Each customer is left exactly once

$$
\displaystyle \sum_{j=1}^{n} x_{ji} = 1 \qquad \forall\, i = 2, \dots, n
$$

**(4)** — Flow conservation: demand at each customer is satisfied by the incoming flow

$$
\displaystyle \sum_{j=1}^{n} f_{ji} - \sum_{j=1}^{n} f_{ij} = D_i \qquad \forall\, i = 2, \dots, n
$$

**(5)** — Flow on each arc is non-negative and bounded by truck capacity

$$
0 \le f_{ij} \le C \cdot x_{ij} \qquad \forall\, i, j = 1, \dots, n
$$

**(6)** — Binary routing variables

$$
x_{ij} \in \{0, 1\} \qquad \forall\, i, j = 1, \dots, n
$$

> **Note on the flow formulation:** Constraint (4) ensures that every customer receives exactly $D_i$ units net from the flow passing through it. Constraint (5) links the flow variable to the routing variable: flow can only travel on active arcs and is bounded by the truck capacity $C$. Together, these constraints implicitly enforce route feasibility, vehicle capacity, and subtour elimination — without the need for auxiliary MTZ ordering variables.

A copy of this formulation is also available as a standalone PDF in this repository.

## Example Instance

The script uses a hardcoded instance featuring:

- **31 nodes** — 1 depot (node 1, coordinates $(50, 50)$) + **30 customers** with predefined $(x, y)$ coordinates in $[1, 100]^2$
- **Euclidean distances** as arc costs: $d_{ij} = \sqrt{(x_j - x_i)^2 + (y_j - y_i)^2}$
- **Truck capacity**: $C = 90$ units
- **Customer demands**: ranging from 2 to 20 units, with a **total demand of 364 units**
- **Minimum vehicles required**: **5 trucks** ($\lceil 364 / 90 \rceil = 5$)

The included results file records the full CPLEX solution. The **5 optimal routes** are:

| Route | Sequence | Load |
|---|---|---|
| 1 | 1 → 13 → 16 → 3 → 6 → 29 → 7 → 28 → 12 → 14 → 1 | 87 |
| 2 | 1 → 20 → 18 → 2 → 22 → 15 → 4 → 10 → 1 | 90 |
| 3 | 1 → 21 → 19 → 11 → 17 → 8 → 23 → 1 | 86 |
| 4 | 1 → 27 → 5 → 1 | 18 |
| 5 | 1 → 30 → 26 → 25 → 31 → 24 → 9 → 1 | 83 |

**Optimal total distance: 679.27** — found in **220.76 seconds**.

## Requirements

Install the required Python packages via pip:

```bash
pip install pyomo numpy pandas
```

**IBM ILOG CPLEX** must also be installed separately on your system. An academic license is available free of charge through the [IBM Academic Initiative](https://www.ibm.com/academic).

## Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/Diego-Fabbri/Vehicle_Routing_Problem_VRP_Py.git
   cd Vehicle_Routing_Problem_VRP_Py
   ```

2. Run the script:
   ```bash
   python Vehicle_Routing_Problem_VRP.py
   ```

## Output

When executed, the script:
- Computes the full $31 \times 31$ Euclidean distance matrix from the hardcoded coordinates
- Builds the MILP model using Pyomo's `ConcreteModel` and prints the full model structure to the console
- Solves it via CPLEX and measures execution time
- Writes the results to `Vehicle_Routing_Problem_VRP_Problem_CLSP_Problem_Results.txt`, including:
  - Execution time in seconds
  - Solver status and termination condition
  - Optimal total travel distance (objective value)
  - Active routing arcs $x[i][j] = 1$ for each truck
  - Flow values $f[i][j]$ on each arc
  - Flow balance verification (inflow − outflow vs demand) for every node including the depot
