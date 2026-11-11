# Project scope and provenance

## What this upload contains

The uploaded `deltarobot.zip` consists of 15 engineering render PNGs, an 11-page technical-drawings PDF, and an embedded CAD export archive (`delta_cell_cad_v2.zip`). It does **not** contain the previously discussed full `delta_cell` robotics application (`core/`, `sim/`, `ros2_ws/`, and pick-and-place task scripts). Do not claim those systems are available in this repository.

The nested CAD archive includes 15 Python code files, a `config/robot.yaml`, a MuJoCo MJCF model and geometry meshes, STEP/STL exports, validation JSON and rendered imagery. This prepared package keeps source, the runnable MJCF assets, drawings and renders, but does **not** include the large generated STEP assemblies and separate part exports. Those remain in the original archive.

## Compatibility adjustment

The source file `analysis/kinematics.py` imports an absent `analysis/params.py`. This package adds a **new compatibility shim** defining `PHI` from the existing YAML configuration, plus explicit estimated task radius/stroke defaults. It is clearly marked as a reconstructed file, not as original collaborator-authored code. Test before comparing new simulation output with historical results.

## Review before publishing

- Confirm your collaborators approve publication of the CAD source, meshes, drawings and images.
- Verify dimensions, clearances and material assumptions against manufactured hardware.
- Choose a license only after obtaining agreement from all relevant rights holders.
- Confirm the MuJoCo model and CadQuery generator run in your intended dependency versions; only Python syntax and a basic kinematics check were evaluated here.
- If you later recover the full application, add it as a **separate source tree** with correct attributions and reproducibility instructions.

## Published repository versus prepared package

The **prepared local ZIP** contains 61 MuJoCo STL meshes, full-resolution PNG renders, and the original drawing PDF. The **GitHub repository** currently publishes the CAD sources, MuJoCo XML, saved reports, 15 lightweight WebP previews, and a reduced-resolution PDF drawing preview; it does **not** yet include those 61 binary STL mesh files. See [UPLOAD_STATUS.md](UPLOAD_STATUS.md) for the complete publication inventory.
