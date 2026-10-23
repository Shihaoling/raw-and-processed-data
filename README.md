# Failure-Aware Truck–Drone Collaborative Routing for Multi-Type Coverage Tasks

This repository provides the instance data, experimental results, and constraint-verification materials accompanying the manuscript.

## Start here

| Directory | Contents |
|---|---|
| [data/](data/) | Data supporting the current revised manuscript: generated and GIS instances, planning and baseline results, failure-recovery results, sensitivity analyses, and validation records. |
| [constraint_verification/](constraint_verification/) | Constraint-reduction report and a complete archive of the verification scripts, inputs, solver logs, results, figures, and reproduction instructions. |
| [archive/legacy_results/](archive/legacy_results/) | Results from the earlier repository release, retained for provenance. Use `data/` for the current manuscript results. |

## Current manuscript data

See [data/README.md](data/README.md) for the file structure, experimental settings, and available run records.

- [Instances](data/instances/): 30 generated instances and the Berlin Hauptbahnhof GIS case.
- [Results](data/results/): SVNS planning, VNS and ALNS baselines, single and multiple drone failures, parameter sensitivity, and the GIS case.
- [Validation](data/validation/): manuscript values, audit records, and the file manifest.

JSON files contain instance inputs, routes, and schedules. CSV files provide result summaries, and XLSX files preserve analysis tables. The data directory contains data and validation records; the constraint-verification scripts are supplied in the archive linked below.

### Instance names

Generated instance files use `instance_<distribution>_<targets>_<area_coverage_node_pairs>.json`. The distribution codes are `C` (clustered), `R` (random), and `RC` (random-clustered). The directories `3`, `6`, `9`, and `12` indicate the number of coverage targets.

For example, `instance_C_3_4.json` has 3 coverage targets and 4 area-coverage node pairs. Its code identifier `C_3_4` corresponds to **C-4-3** in the manuscript, where the two counts appear in the opposite order.

## Constraint verification

- [Read the report (PDF)](constraint_verification/Constraint_Reduction_and_COPT_Verification.pdf)
- [Download the complete verification archive](constraint_verification/round2_constraint_verification.zip)
- [Check the SHA-256 digests](constraint_verification/SHA256SUMS.txt)

Extract the archive and follow its README to inspect the records or run the checks. It includes the row-by-row audit of 105 stored SVNS plans, toy-instance constraint-removal tests, and archived COPT runs on the three small instances. The report and code use R1 constraint numbers; the included `equation_crosswalk.csv` maps these to the current manuscript.

The archive preserves historical solver runs. Its README distinguishes the original comparisons from confirmation runs and records the bounds, gaps, and dependencies. Non-solver checks use the included inputs; re-solving requires a user-supplied COPT installation and appropriate license.

## GIS attribution

The Berlin Hauptbahnhof geographic features are derived from OpenStreetMap. Map data © OpenStreetMap contributors; see the [OpenStreetMap attribution and license information](https://www.openstreetmap.org/copyright). The source geographic records and model instance are provided in [data/instances/gis/](data/instances/gis/).

## Previous directory names

The former top-level directories `current_revision_data/`, `round2_constraint_verification/`, and `results/` retain README pointers to `data/`, `constraint_verification/`, and `archive/legacy_results/`, respectively.

See [directory changes](DIRECTORY_CHANGES.md) for the path mapping and a link to the complete pre-reorganization snapshot.
