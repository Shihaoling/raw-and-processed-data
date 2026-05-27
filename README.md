# Raw and Processed Data

This repository contains the raw result files and processed data for a **Self-Learning Variable Neighborhood Search (SVNS)** algorithm applied to a drone-truck collaborative inspection/routing problem. The algorithm co-optimizes routes for a ground truck and multiple drones to minimize the total mission completion time (makespan).

---

## Repository Structure

```
results/
├── basic_instance/          # Main benchmark results (SL-VNS)
│   ├── 3/                   # 3 coverage areas, 4 operational nodes
│   ├── 6/                   # 6 coverage areas, 10/20/30 nodes
│   ├── 9/                   # 9 coverage areas, 10/20/30 nodes
│   └── 12/                  # 12 coverage areas, 10/20/30 nodes
├── GA/                      # Genetic Algorithm baseline results
├── greedy/                  # Greedy algorithm baseline results
├── drone_failure/           # Drone failure recovery experiments
├── drone_related_instance/
│   ├── drone_num/           # Sensitivity to number of drones (1–5)
│   └── drone_endurance/     # Sensitivity to drone endurance (5–20)
├── Heuristics_parameter/    # Algorithm parameter sensitivity study
│   ├── local_search_num/
│   ├── local_search_temperature_update_frequency/
│   ├── shaking_resets_num/
│   ├── stop_num/
│   └── temperature_decay/
├── recovery/                # Route recovery scenario results
└── solution_details.xlsx    # Consolidated solution spreadsheet
```

---

## Instance Types

Three standard instance types are used, varying in the spatial distribution of nodes:

| Type | Description |
|------|-------------|
| **C** | Clustered — nodes grouped in geographic clusters |
| **RC** | Random-Clustered — mixed distribution |
| **R** | Random — uniformly random node placement |

Instance naming convention: `result_{Type}_{CoverageAreas}_{OperationalNodes}.json`  
Example: `result_C_9_10.json` → Clustered, 9 coverage areas, 10 operational nodes.

---

## Main Results (SL-VNS on Basic Instances)

Results are stored in `results/basic_instance/`. Each JSON file contains the optimized truck/drone routes and key performance metrics.

| Instance | SL-VNS (time) | Improvement from init | Iterations |
|----------|:-------------:|:---------------------:|:----------:|
| C\_3\_4   | 7.686  | 33.9% | 42.2  |
| C\_6\_10  | 8.279  | 59.4% | 60.8  |
| C\_6\_20  | 8.983  | 63.4% | 65.8  |
| C\_6\_30  | 8.715  | 70.4% | 52.0  |
| C\_9\_10  | 9.818  | 59.9% | 88.2  |
| C\_9\_20  | 9.852  | 69.0% | 73.8  |
| C\_9\_30  | 11.597 | 70.4% | 81.0  |
| C\_12\_10 | 14.388 | 47.7% | 65.0  |
| C\_12\_20 | 15.480 | 57.1% | 76.8  |
| C\_12\_30 | 17.085 | 61.9% | 94.0  |
| RC\_3\_4  | 7.953  | 38.4% | 36.0  |
| RC\_6\_10 | 8.675  | 56.7% | 69.2  |
| RC\_6\_20 | 7.969  | 66.1% | 56.6  |
| RC\_6\_30 | 8.596  | 68.3% | 60.4  |
| RC\_9\_10 | 9.747  | 55.3% | 116.8 |
| RC\_9\_20 | 10.063 | 63.5% | 96.8  |
| RC\_9\_30 | 10.653 | 70.1% | 72.4  |
| RC\_12\_10| 13.563 | 54.6% | 121.4 |
| RC\_12\_20| 15.138 | 60.7% | 142.8 |
| RC\_12\_30| 19.451 | 55.6% | 105.2 |
| R\_3\_4   | 7.724  | 43.5% | 42.6  |
| R\_6\_10  | 8.034  | 61.5% | 61.0  |
| R\_6\_20  | 8.876  | 64.8% | 69.0  |
| R\_6\_30  | 9.378  | 68.2% | 65.0  |
| R\_9\_10  | 11.708 | 58.8% | 112.0 |
| R\_9\_20  | 13.497 | 62.3% | 109.2 |
| R\_9\_30  | 14.571 | 66.2% | 85.8  |
| R\_12\_10 | 17.739 | 46.9% | 102.2 |
| R\_12\_20 | 19.915 | 54.9% | 88.0  |
| R\_12\_30 | 19.484 | 64.4% | 104.2 |

*Time unit: minutes. Improvement = relative reduction from initial greedy solution to SL-VNS optimum.*

---

## Comparison with Baselines

SL-VNS is compared against a **Greedy** algorithm and a **Genetic Algorithm (GA)** on all 30 benchmark instances.

| Instance | SL-VNS | Greedy | GA | vs Greedy | vs GA |
|----------|:------:|:------:|:--:|:---------:|:-----:|
| C\_3\_4   | 7.686  | 9.899  | 9.336  | −22.4% | −17.7% |
| C\_6\_10  | 8.279  | 18.503 | 17.170 | −55.3% | −51.8% |
| C\_6\_20  | 8.983  | 21.904 | 22.266 | −59.0% | −59.7% |
| C\_6\_30  | 8.715  | 20.060 | 27.516 | −56.6% | −68.3% |
| C\_9\_10  | 9.818  | 19.519 | 23.256 | −49.7% | −57.8% |
| C\_9\_20  | 9.852  | 27.727 | 26.412 | −64.5% | −62.7% |
| C\_9\_30  | 11.597 | 26.642 | 33.908 | −56.5% | −65.8% |
| C\_12\_10 | 14.388 | 24.794 | 28.191 | −42.0% | −49.0% |
| C\_12\_20 | 15.480 | 27.404 | 37.504 | −43.5% | −58.7% |
| C\_12\_30 | 17.085 | 36.569 | 43.849 | −53.3% | −61.0% |
| RC\_3\_4  | 7.953  | 12.152 | 10.847 | −34.6% | −26.7% |
| RC\_6\_10 | 8.675  | 18.516 | 17.771 | −53.1% | −51.2% |
| RC\_6\_20 | 7.969  | 19.127 | 21.121 | −58.3% | −62.3% |
| RC\_6\_30 | 8.596  | 18.449 | 25.596 | −53.4% | −66.4% |
| RC\_9\_10 | 9.747  | 20.931 | 23.596 | −53.4% | −58.7% |
| RC\_9\_20 | 10.063 | 24.732 | 25.672 | −59.3% | −60.8% |
| RC\_9\_30 | 10.653 | 27.677 | 40.268 | −61.5% | −73.5% |
| RC\_12\_10| 13.563 | 28.137 | 32.879 | −51.8% | −58.7% |
| RC\_12\_20| 15.138 | 33.739 | 39.515 | −55.1% | −61.7% |
| RC\_12\_30| 19.451 | 39.049 | 48.171 | −50.2% | −59.6% |
| R\_3\_4   | 7.724  | 12.595 | 15.456 | −38.7% | −50.0% |
| R\_6\_10  | 8.034  | 19.432 | 17.838 | −58.7% | −55.0% |
| R\_6\_20  | 8.876  | 17.743 | 24.259 | −50.0% | −63.4% |
| R\_6\_30  | 9.378  | 18.627 | 27.525 | −49.7% | −65.9% |
| R\_9\_10  | 11.708 | 27.418 | 28.780 | −57.3% | −59.3% |
| R\_9\_20  | 13.497 | 31.223 | 37.911 | −56.8% | −64.4% |
| R\_9\_30  | 14.571 | 33.745 | 43.311 | −56.8% | −66.4% |
| R\_12\_10 | 17.739 | 27.845 | 30.856 | −36.3% | −42.5% |
| R\_12\_20 | 19.915 | 38.436 | 39.237 | −48.2% | −49.2% |
| R\_12\_30 | 19.484 | 40.129 | 45.850 | −51.4% | −57.5% |

**Average improvement: SL-VNS reduces makespan by 51.2% vs Greedy and 56.9% vs GA across all 30 instances.**

---

## Drone Failure Recovery

Results in `results/drone_failure/` contain reconstructed routes after a simulated drone failure mid-mission. A total of 31 instances (C/RC/R × multiple sizes) were tested.

| Instance | Reconstructed Time |
|----------|--------------------|
| C\_3\_4   | 7.686  |
| C\_6\_10  | 8.180  |
| C\_6\_20  | 11.908 |
| C\_6\_30  | 14.416 |
| C\_9\_10  | 12.661 |
| C\_9\_20  | 19.412 |
| C\_9\_30  | 14.597 |
| C\_12\_10 | 16.248 |
| C\_12\_20 | 19.262 |
| C\_12\_30 | 20.047 |
| RC\_3\_4  | 9.424 / 11.475 |
| RC\_6\_10 | 10.713 |
| RC\_6\_20 | 9.929  |
| RC\_6\_30 | 11.245 |
| RC\_9\_10 | 11.611 |
| RC\_9\_20 | 10.468 |
| RC\_9\_30 | 14.105 |
| RC\_12\_10| 12.089 |
| RC\_12\_20| 25.234 |
| RC\_12\_30| 19.340 |
| R\_3\_4   | 12.509 |
| R\_6\_10  | 14.090 |
| R\_6\_20  | 10.684 |
| R\_6\_30  | 10.563 |
| R\_9\_10  | 15.601 |
| R\_9\_20  | 16.523 |
| R\_9\_30  | 16.095 |
| R\_12\_10 | 22.894 |
| R\_12\_20 | 25.309 |
| R\_12\_30 | 26.240 |

---

## Drone Number Sensitivity

Results in `results/drone_related_instance/drone_num/` test instances C\_9\_10, RC\_9\_10, and R\_9\_10 with 1 to 5 drones (5 runs each).

| Drones | C\_9\_10 (mean) | RC\_9\_10 (mean) | R\_9\_10 (mean) |
|:------:|:---------------:|:----------------:|:---------------:|
| 1      | 18.885          | 17.860           | 21.994          |
| 2      | 15.006          | 13.910           | 17.444          |
| 3      | 14.105          | 13.218           | 16.248          |
| 4      | 12.860          | 11.214           | 14.716          |
| 5      | 11.537          | 10.799           | 14.886          |

*Adding more drones consistently reduces makespan; diminishing returns observed beyond 4 drones.*

---

## Drone Endurance Sensitivity

Results in `results/drone_related_instance/drone_endurance/` test instance C\_9\_10 with drone endurance values of 5, 10, 15, and 20 minutes.

| Endurance (min) | Global Optimal Time |
|:---------------:|:-------------------:|
| 5               | 11.339              |
| 10              | 10.771              |
| 15              | 11.568              |
| 20              | 10.590              |

---

## Algorithm Parameter Sensitivity

All parameter sensitivity experiments use instance C\_9\_10. Results are in `results/Heuristics_parameter/`.

### Local Search Iterations (`local_search_num`)

| local\_search\_num | Global Optimal Time |
|:-----------------:|:-------------------:|
| 100               | 15.079              |
| 150               | 14.046              |
| 200               | 15.260              |

### Temperature Update Frequency (`local_search_temperature_update_frequency`)

| Frequency | Global Optimal Time |
|:---------:|:-------------------:|
| 10        | 10.867              |
| 20        | 11.300              |
| 30        | 11.433              |
| 40        | 11.507              |

### Shaking Resets (`shaking_resets_num`)

| shaking\_resets\_num | Global Optimal Time |
|:--------------------:|:-------------------:|
| 5                    | 14.908              |
| 10                   | 14.344              |
| 15                   | 13.638              |
| 20                   | 14.000              |
| 25                   | 15.006              |

### Stop Criterion (`stop_num`)

| stop\_num | Global Optimal Time |
|:---------:|:-------------------:|
| 20        | 14.014              |
| 25        | 14.660              |
| 30        | 16.381              |
| 35        | 14.945              |
| 40        | 14.476              |
| 45        | 14.352              |

### Temperature Decay (`temperature_decay`)

| Decay | Global Optimal Time |
|:-----:|:-------------------:|
| 0.90  | 14.203              |
| 0.92  | 14.780              |
| 0.94  | 14.786              |
| 0.96  | 14.470              |
| 0.97  | 13.957              |
| 0.98  | 14.326              |
| 0.99  | 14.192              |

---

## Recovery Scenarios

Results in `results/recovery/` contain route solutions for partial-coverage recovery scenarios.

| Instance   | Total Time |
|------------|:----------:|
| C\_3\_4    | 7.280      |
| C\_6\_10   | 25.665     |
| RC\_3\_4   | 11.275     |
| RC\_6\_10  | 10.696     |
| RC\_6\_20  | 12.772     |
| R\_3\_4    | 9.463      |
| R\_6\_10   | 13.725     |
| R\_6\_20   | 15.078     |

---

## Result File Format

Each JSON result file contains the following fields:

| Field | Description |
|-------|-------------|
| `truck_route` | Ordered list of truck stop nodes |
| `drone_route` | Nested dict: drone → sortie → ordered node list |
| `global_optimal_time` | Best makespan found (minutes) |
| `global_optimal_time_var` | Variance of best makespan across runs |
| `improvement` | Relative improvement from initial solution |
| `init_time` | Initial (greedy) solution makespan |
| `iter_num` | Number of VNS iterations executed |
| `time_elapsed` | Wall-clock computation time (seconds) |
| `total_local_search_weight` | Final self-learned neighborhood weights |
| `shaking_method_first_num` | Times each shaking operator was first-best |
