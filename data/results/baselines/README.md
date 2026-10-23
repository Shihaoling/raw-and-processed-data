# VNS and ALNS baseline records

| Path | Contents |
|---|---|
| [dd15_rerun/](dd15_rerun/) | VNS and ALNS run records with drone endurance `DD = 15`, matching the current generated-instance setting. |
| [original_dd5/](original_dd5/) | Earlier VNS and ALNS run records with `DD = 5`, retained for provenance. |
| [baseline_common_model_evaluation.csv](baseline_common_model_evaluation.csv) | Baseline plans evaluated with the common schedule model; the manuscript tables use the best-of-five objective values from this evaluation. |

Each run directory contains 30 instance records per algorithm. Original JSON filenames,
fields, and numerical results are preserved. The `V` suffix in a filename denotes fleet
size, not the drone-endurance setting.

The former top-level `ALNS/` and `VNS/` subdirectories are now under `original_dd5/`.
Historical source paths inside the records are retained as provenance.
