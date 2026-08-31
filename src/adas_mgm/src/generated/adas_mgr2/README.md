# ADAS_MGR2 four-state generated runtime

This directory contains only the C runtime needed to compile the four-state
Simulink model `ADAS_MGR2` (model version 1.88, generated with Simulink Coder
R2026a on 2026-08-24).

The generated state machine covers:

- `LANE`, `WAYPOINT`, `AVOID`, and `PARKING` states and their path sources
- lane-confidence hysteresis and cross-track return gating
- stop, avoid, and GPS-only zone inputs
- avoid entry/exit, return hold, timeout, TTC stop, and narrow-path speed caps
- parking completion, GPS end, and wrong-way latch/release behavior
- acceleration-zone, traffic-stop, and E-stop longitudinal behavior
- reference hold/blend, stale-frame compensation, and rate limiting

Model version 1.88 does **not** contain the production core's rear-escape
extension (`estop_rear_clear`, `escape_after_cycles`, `v_escape`, and related
escape sequencing). That feature was added to the C++ core after this model was
generated. The generated backend must therefore run with rear escape disabled
(`escape_after_cycles=0`) and fail at startup rather than silently accepting an
unsupported escape configuration. Regenerate the model with the rear-escape
contract before claiming parity for that feature.

Included:

- model API and bus definitions (`ADAS_MGR2.c/.h`, private/types headers)
- generated fixed-width runtime types
- non-finite helper sources required by `ADAS_MGR2_initialize()`

Excluded because they are not runtime inputs:

- Visual Studio make, batch, and response files
- MATLAB build metadata, trace reports, and HTML assets
- generated host/setup project files

The generated files are **not MIT-licensed package code**. They retain the
MathWorks Academic License notice that limits use to teaching, academic
research, and course requirements at degree-granting institutions. They were
generated from the team's own model and are included for this noncommercial
academic project. The MathWorks Program Offering Guide permits copying and
deploying Coder-generated forms outside the Programs, subject to the applicable
license offering. Keep all generated notices intact and do not reuse these
files for commercial, government, or other organizational work without first
confirming the applicable rights.

See the current [MathWorks Program Offering Guide](https://www.mathworks.com/help/pdf_doc/offering/offering.pdf),
Part Two, Section 3.2 (Coder Programs).

`GeneratedMgmAdapter` uses the generated code only in the explicitly enabled
generated backend and in back-to-back tests. The runtime is compiled only on
Linux x86-64 with a GNU/Clang C compiler. Its global API is non-reentrant, so
the adapter uses one instance on one thread.

Model version 1.88 still contains the legacy commanded-velocity stale-frame
compensation. The repository keeps these generated files unchanged from the
model output; `GeneratedMgmAdapter` restores the pre-step reference on a stale
repeat so the deployed contract is a 100 ms perception/GNSS hold. Keeping the
compatibility rule in maintained adapter code prevents a future model
regeneration from silently restoring the legacy behavior. Back-to-back tests
cover this override.

To update this directory after regenerating the model, replace exactly the
twelve runtime files already present here, normalize them to LF, and run
`colcon test --packages-select adas_mgm` on a supported test host. Four-state
back-to-back parity must report zero mismatched supported ticks. The generated
header reports `Validation result: Not run`; repository tests do not replace a
MathWorks code-generation validation report.
