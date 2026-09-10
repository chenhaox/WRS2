"""Headless assembly geometry in metres, without robot or viewer startup."""
from .model import (Assembly, AssemblyState, ContactAnalysis, ContactConfig,
                    ContactPatch, GeometryConfig, MatingRelation, MeshData,
                    Part, PreparedMesh, Region, SurfacePatch)
from .io import load_assembly, save_assembly, save_report

__all__ = ['Assembly', 'AssemblyState', 'ContactAnalysis', 'ContactConfig',
           'ContactPatch', 'GeometryConfig', 'MatingRelation', 'MeshData',
           'Part', 'PreparedMesh', 'Region', 'SurfacePatch', 'load_assembly',
           'save_assembly', 'save_report', 'analyze_pair', 'analyze_contacts']


def __getattr__(name):
    if name in ('analyze_pair', 'analyze_contacts'):
        from .contact import analysis
        return getattr(analysis, name)
    raise AttributeError(name)
