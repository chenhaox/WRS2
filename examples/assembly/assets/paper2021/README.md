# Paper example inputs

These 13 STL files and 12 JSON scenes were copied without modification from
the user's legacy `assembly_planner` checkout. `provenance.json` records each
original path, SHA-256 digest and source commit. Units are explicitly taken as
millimetres; the STL format itself does not encode units.

Run the examples directly; the legacy checkout is not needed at runtime.
`tools/import_paper_example_assets.py` is the optional import utility. It reads
JSON and copies geometry; it does not import old Python modules or unpickle data.

`alframe.stl`, `datainfo0209`, and `datainfo0209_2` were absent from the checked
source commit. Their examples are labelled reconstructions. The random scenes
use recorded seeds and are replacements, not the paper's unpublished samples.

The nominal example mode repairs the known Soma grid's triangulation and rounded
angles in memory. Raw bytes in this directory remain untouched. See
[coverage, commands and limitations](../../../../docs/assembly_planner/paper-examples.zh.md).
