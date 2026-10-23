# Round 2 constraint verification

Materials supporting the constraint-verification response for **Failure-Aware Truck–Drone Collaborative Routing for Multi-Type Coverage Tasks**, manuscript AMMOD-D-26-02709.

- [Read the verification report (PDF)](Constraint_Reduction_and_COPT_Verification.pdf)
- [Download the complete verification archive](round2_constraint_verification.zip)
- [SHA-256 checksums](SHA256SUMS.txt)

Download and extract `round2_constraint_verification.zip` to access the report in PDF and editable Word formats, the mathematical constraint crosswalk, scripts, model inputs, archived solver results, solver logs, figures, and reproduction instructions. The archive includes an English README and a checksum manifest for every file.

The materials support the reduced first-stage formulation: row-by-row checks of 105 stored SVNS plans, toy-instance removal tests, and the COPT comparisons on the three small instances. The archive preserves the historical optimization records; preparing this release did not run COPT or produce new optimization results. Public copies were checked by reproducing the audit tables and rechecking the archived plans. Small-instance runs retain their original bounds and nonzero gaps and are not described as proofs of global optimality.

The report and code use R1 constraint numbers. `equation_crosswalk.csv` maps them to the current manuscript. The README distinguishes the original 1800-second comparisons, the C-4-3 observations taken from a 2400-second log, and the RC-4-3 confirmation runs.

The numerical model/audit code concerns the first stage. Re-solving requires a user-supplied COPT installation and appropriate license; no license or solver binary is distributed. Non-solver checks can be run with the included dependencies and inputs.
