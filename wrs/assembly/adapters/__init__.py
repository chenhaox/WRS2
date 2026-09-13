"""Convert external data to the planner's metre-based geometry and state.

legacy converts ASP_OLD JSON/STL units and rotation conventions.
wrs_scene converts WRS visual meshes/poses and creates display/robot objects.
Both are used by current tools and examples; neither implements a solver.
Import individual functions from these modules so optional WRS imports stay lazy.
"""
