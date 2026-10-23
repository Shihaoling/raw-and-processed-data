# Manuscript data

This directory contains the instance data and experimental outputs supporting the revised
manuscript *Failure-Aware Truck-Drone Collaborative Routing for Multi-Type Coverage Tasks*.
The files were curated from the records used for the current manuscript revision.
Earlier baseline runs are retained separately for provenance. Constraint-verification
scripts and solver logs are provided in [constraint_verification/](../constraint_verification/).

## Directory structure

```text
data/
├── instances/
│   ├── generated/                 Generated C, R, and RC coverage instances
│   └── gis/                       Berlin Hauptbahnhof GIS source and model instance
├── results/
│   ├── planning_svns/             SVNS planning results and retained run records
│   ├── baselines/                 VNS and ALNS result records
│   ├── single_failure/            Single-failure recovery outputs
│   ├── parameter_sensitivity/     Fleet-size and algorithm-parameter results
│   ├── simultaneous_failures/     Two-drone simultaneous-failure outputs
│   ├── sequential_failures/       Sequential multi-drone failure outputs
│   └── gis_case/                  GIS planning and recovery schedules/results
└── validation/                    Manuscript values, audits, and file manifest
```

## Instance data

### Generated instances

`instances/generated/{3,6,9,12}/` contains 30 JSON instances. The directory name is the
number of coverage targets. File names follow
`instance_{distribution}_{targets}_{area_coverage_node_pairs}.json`, where:

- `C` denotes clustered instances;
- `R` denotes randomly distributed instances; and
- `RC` denotes random-clustered instances.

Each file contains target centers, types and geometries, coverage-node coordinates, and
truck-accessible candidate-node coordinates. The vehicle and timing settings used with
this geometry are described below.

### GIS instance

`instances/gis/` contains the Berlin Hauptbahnhof instance used in the illustrative case:

- `instance_berlin_hbf_svns.json`: normalized optimization instance with node coordinates;
- `svns_notebook_input_summary.json`: summarized model input;
- `source/berlin_hbf_features.geojson`: extracted geographic features;
- `source/berlin_hbf_overpass.json`: source Overpass response; and
- `source/berlin_hbf_base_instance.json`: geographic-to-model construction record.

The geographic features are derived from OpenStreetMap and remain subject to OpenStreetMap
attribution and licensing requirements.

## Experimental settings

The current manuscript's generated-instance planning, baseline, recovery, and fleet-size experiments use:

- truck speed: 20 coordinate-distance units per model time unit;
- drone speed: 30 coordinate-distance units per model time unit;
- launch time: 0.1;
- retrieval time: 0.1;
- drone endurance: 15; and
- truck endurance: 200.

The GIS case uses launch/retrieval times of 0.05, drone endurance of 10, and two drones.

## Result data

- `results/planning_svns/` contains the 30 instance-level SVNS results, the consolidated CSV,
  route-record provenance, and complete five-run records for the six re-solved instances.
  For the other 24 instances evaluated before the revision, the retained records contain the
  mean and variance across five runs but not the objective value of each individual run.
- `results/baselines/` contains the VNS and ALNS run records and their common-model
  evaluation. `dd15_rerun/{ALNS,VNS}/` contains the runs with drone endurance 15.
  `original_dd5/{ALNS,VNS}/` preserves the earlier runs with drone endurance 5.
  `baseline_common_model_evaluation.csv` contains the baseline plans evaluated with
  the common schedule model; its best-of-five values are used in the manuscript tables.
  See the [baseline data guide](results/baselines/README.md).
- `results/single_failure/` contains all 30 recovered plans, event states, absolute schedules,
  reconstruction records, and audit fields.
- `results/parameter_sensitivity/` contains fleet-size runs and summaries, sortie statistics,
  and the retained parameter-sensitivity spreadsheets.
- `results/simultaneous_failures/` contains 27 scenario records and their CSV/XLSX summaries.
- `results/sequential_failures/` contains the 18 reported sequential-failure records, additional
  generated records, and their CSV/XLSX summaries.
- `results/gis_case/` contains the nominal and recovered schedules and result summaries.

JSON records preserve detailed routes and schedules. CSV files provide consolidated,
machine-readable summaries, while XLSX files retain the tabular records used during analysis.

## Validation and provenance

`validation/manuscript_numbers.json` records the numerical values used in the manuscript.
The audit JSON files document the data correction and node-level validation checks.
[`validation/file_manifest.csv`](validation/file_manifest.csv) lists every file in `data/`
except the manifest itself, with its path relative to `data/`, byte size, and SHA-256 digest.
The digests describe the bytes stored in GitHub, including LF line endings for text files.
Use a ZIP download or a checkout that preserves those line endings when checking hashes.
The [previous manifest](../archive/previous_file_manifest.csv) is retained for provenance;
its 62 affected entries describe the pre-upload CRLF bytes instead of the stored LF bytes.

Every reported recovered schedule was independently checked for chronological consistency,
retrieval before relaunch, fixed execution history, endurance feasibility, and complete
coverage.
