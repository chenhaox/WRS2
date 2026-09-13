"""Optional contact previews; scene construction and HTML writing are explicit."""

from .contact import (
    COLORS as COLORS,
    build_wrs_scene as build_wrs_scene,
    preview_case as preview_case,
    write_contact_html as write_contact_html,
)

__all__ = ["COLORS", "build_wrs_scene", "preview_case", "write_contact_html"]
