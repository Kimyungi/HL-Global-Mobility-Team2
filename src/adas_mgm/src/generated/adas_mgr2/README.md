# ADAS_MGR2 generated runtime

This directory contains only the C runtime needed to compile Simulink model
`ADAS_MGR2` (model version 1.55, generated with Simulink Coder R2026a on
2026-08-17).

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

They are used by `GeneratedMgmAdapter` only for back-to-back verification and
are compiled only when `BUILD_TESTING` is enabled on Linux x86-64 with a
GNU/Clang C compiler. The production ROS 2 node continues to call the
hand-maintained, ROS-independent `mgm_core` library. The generated API is
global and non-reentrant, so it must not be called from multiple instances or
threads.

To update this directory after regenerating the model, replace the same runtime
file set and run `colcon test --packages-select adas_mgm` on a supported x86-64
test host. The parity test must report zero mismatched ticks before the
generated model or C++ core is accepted. The generated header reports
`Validation result: Not run`; this back-to-back test does not replace a
MathWorks code-generation validation report.
