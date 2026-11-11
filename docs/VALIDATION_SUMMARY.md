# Existing model-validation results

The source archive includes `cad/out/mujoco/validation.json` and `cad/out/report.json`. Values below are **recorded results from the supplied archive**, not measurements or simulations rerun during GitHub preparation.

- Reference platform: **(0, 0, −240) mm**.
- Model: **11 bodies**, **10 joints**, **6 equality constraints**, **3 actuators**.
- Max equality-constraint residual during the saved move tests: **0.02298 mm**.
- Four supplied move targets: `(50,30,-260)`, `(-80,-86.5,-240)`, `(75,86.5,-215)`, and `(0,0,-300)` mm.
- Reported target-position errors were **0.097, 0.073, 0.088, and 0.084 mm**, respectively.
- Reported platform tilt stayed below **0.07°** in those four recorded moves.

See [validation JSON](../cad/out/mujoco/validation.json) for the full recorded values. The simulation still requires validation after rebuilding from source and does not establish real-hardware tracking accuracy.
