# Directory changes — 24 October 2026

The repository is organized by the role of each collection in the current manuscript.
The repository URL remains unchanged.

| Previous path | Current path |
|---|---|
| `current_revision_data/` | [data/](data/) |
| `current_revision_data/results/baselines/ALNS/` | [data/results/baselines/original_dd5/ALNS/](data/results/baselines/original_dd5/ALNS/) |
| `current_revision_data/results/baselines/VNS/` | [data/results/baselines/original_dd5/VNS/](data/results/baselines/original_dd5/VNS/) |
| `round2_constraint_verification/` | [constraint_verification/](constraint_verification/) |
| `results/` | [archive/legacy_results/](archive/legacy_results/) |

All other paths inside these collections keep their relative names. Dataset values,
result records, solver logs, and the published verification PDF and ZIP are unchanged.
Source-path strings embedded in historical records remain as provenance.

The root and collection READMEs now describe the current structure. The data manifest
was rebuilt from the stored bytes, correcting 62 entries affected by the earlier
CRLF-to-LF conversion. The verification checksums were updated for its revised README.
Five macOS `.DS_Store` files and two generated Python bytecode files were removed;
`.gitignore` prevents them from being added again.

The three former top-level directories retain README pointers. These preserve directory
entry points, but do not automatically redirect individual old file links. For an older
file-level reference, use the mapping above or the
[complete pre-reorganization snapshot](https://github.com/Shihaoling/raw-and-processed-data/tree/4211439c4ae4225932b5f8f3f531a152410364fd).
