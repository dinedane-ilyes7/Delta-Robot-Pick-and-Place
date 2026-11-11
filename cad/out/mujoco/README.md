# MuJoCo model — assets and reproduction notes

This directory publishes the generated `delta_cell.xml` model plus `bodies.json` and `validation.json` saved in the CAD-v2 project archive.

**The 61 binary STL meshes required by the `meshdir="meshes"` setting are not yet hosted in GitHub.** They remain in the project's original offline ZIP. Running `mujoco.viewer` on the XML alone will fail until `cad/out/mujoco/meshes/` is populated.

To reconstruct the meshes from the published CAD source, install the dependencies listed in `requirements-cad.txt` (CadQuery/OpenCascade and others), then run from the project root:

```bash
python cad/build.py
python -m mujoco.viewer --mjcf cad/out/mujoco/delta_cell.xml
```

This build procedure **has not been tested end-to-end in the GitHub publication workflow**. It may require resolving environment-specific package versions. The archived simulation results are not new simulations rerun for this publication.

For source and methodology, see [CAD notes](../../README.md) and the [repository scope](../../../docs/PROJECT_SCOPE.md).
