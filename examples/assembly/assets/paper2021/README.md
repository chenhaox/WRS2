# Paper example inputs

These 13 STL files and 12 JSON scenes were copied without modification from
the user's legacy `assembly_planner` checkout. `provenance.json` records each
original path, SHA-256 digest and source commit. Units are explicitly taken as
millimetres; the STL format itself does not encode units.

Run the examples directly; the legacy checkout is not needed at runtime.
Examples now load the 34 native `wrs.assembly/1` files under `assemblies/`.
Their inline geometry and pose translations use metres; rotations are matrices.
The two Burr nominal manifests include declared slot-plane alignment and a
closed-boundary rebuild (maximum source-vertex change below 0.3 mm). Raw Burr
meshes/manifests preserve the original interference. These corrections are a
nominal fit model, not recovered author CAD; see
[Burr diagnosis and repair](../../../../docs/assembly_planner/burr-fix.zh.md).
Raw and nominal modes remain separate, including provenance and corrections.
Six unavailable raw modes still raise explicitly instead of being reconstructed.
`tools/import_paper_example_assets.py` is the optional import utility. It reads
JSON and copies geometry; it does not import old Python modules or unpickle data.

`alframe.stl`, `datainfo0209`, and `datainfo0209_2` were absent from the checked
source commit. Their examples are labelled reconstructions. The random scenes
use recorded seeds and are replacements, not the paper's unpublished samples.

The one-time converter repairs the known Soma grid's triangulation and rounded
angles before writing nominal manifests. To regenerate them from the archived
inputs, run `python -m tools.assembly.paper_assets` from the repository root.
Runtime examples neither import this tool nor depend on the old JSON format.
Raw bytes in `scenes/` and `meshes/` remain untouched and are used to reproduce
and verify the conversion. See
[coverage, commands and limitations](../../../../docs/assembly_planner/paper-examples.zh.md).
