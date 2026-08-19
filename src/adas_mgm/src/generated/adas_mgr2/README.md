# ADAS_MGR2 LANE/WAYPOINT generated runtime

This directory contains only the C runtime needed to compile the two-state
Simulink model `ADAS_MGR2` (model version 1.68, generated with Simulink Coder
R2026a on 2026-08-18).

The experiment is intentionally limited to:

- `LANE` state and lane-path output
- `WAYPOINT` state and GPS-path output
- confidence hysteresis and cross-track return gating
- acceleration-zone, traffic-stop, and E-stop longitudinal behavior
- reference hold/blend, stale-frame compensation, and rate limiting

`AVOID` and `PARKING` inputs remain in the shared bus ABI but are outside this
model and test. The parity fixture keeps their transition inputs false. The
production ROS 2 node and its existing `mgm_core` implementation are not
changed or linked to this generated model.

`gps_at_end` is also held false in the parity fixture. Version 1.68 treats it
as a raw stop input, while the production core deliberately latches a valid
track end until an explicit release. That policy difference is not part of the
two-state switching experiment and must not be presented as verified parity.

`gps_heading_valid` is held false as well. Version 1.68 does not implement the
production core's WAYPOINT wrong-way counters or latch, so heading-valid and
wrong-way behavior is outside this parity scope. "WAYPOINT parity" here covers
the switching, path, and longitudinal items listed above, not every production
WAYPOINT safety policy.

Included:

- model API and bus definitions (`ADAS_MGR2.c/.h`, private/types headers)
- generated fixed-width runtime types
- non-finite helper sources required by `ADAS_MGR2_initialize()`

Excluded because they are not runtime inputs:

- generated example `ert_main.c`
- Visual Studio make/batch/response files
- MATLAB build metadata and HTML trace output

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

The generated code is used by `GeneratedMgmAdapter` only for a two-state
back-to-back test and is compiled only when `BUILD_TESTING` is enabled on Linux
x86-64 with a GNU/Clang C compiler. Its global API is non-reentrant, so the
test uses one instance on one thread.

To update this directory after regenerating the model, replace the same runtime
file set and run `colcon test --packages-select adas_mgm` on a supported test
host. The LANE/WAYPOINT parity test must report zero mismatched ticks. The
generated header reports `Validation result: Not run`; this back-to-back test
does not replace a MathWorks code-generation validation report.
