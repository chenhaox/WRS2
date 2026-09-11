"""Headless assembly geometry in metres, without robot or viewer startup."""
from .model import (Assembly, AssemblyState, ContactAnalysis, ContactConfig,
                    ContactPatch, GeometryConfig, MatingRelation, MeshData,
                    Part, PreparedMesh, Region, SurfacePatch)
from .io import load_assembly, save_assembly, save_report

__all__ = ['Assembly', 'AssemblyState', 'ContactAnalysis', 'ContactConfig',
           'ContactPatch', 'GeometryConfig', 'MatingRelation', 'MeshData',
           'Part', 'PreparedMesh', 'Region', 'SurfacePatch', 'load_assembly',
           'save_assembly', 'save_report', 'analyze_pair', 'analyze_contacts',
           'ContactModel', 'ContactAnalyzer', 'ContactBackend', 'MeshContactBackend',
           'SDFContactBackend', 'SDFConfig', 'GridSDF', 'SignedDistanceField', 'SDFSamples', 'SDFCollisionChecker',
           'ToleranceContactPolicy', 'ContactGraph', 'build_contact_graph', 'ConstraintConfig',
           'LocalConstraints', 'MotionCandidates', 'contact_constraints', 'candidate_motions', 'rebase_twist']


def __getattr__(name):
    if name in ('ContactGraph', 'build_contact_graph'):
        from .contact import graph
        return getattr(graph, name)
    if name in ('ConstraintConfig', 'LocalConstraints', 'MotionCandidates',
                'contact_constraints', 'candidate_motions', 'rebase_twist'):
        from . import constraints
        return getattr(constraints, name)
    if name == 'ToleranceContactPolicy':
        from .contact.policy import ToleranceContactPolicy
        return ToleranceContactPolicy
    if name == 'SDFCollisionChecker':
        from .contact.sdf_collision import SDFCollisionChecker
        return SDFCollisionChecker
    if name == 'ContactModel':
        from .contact.models import ContactModel
        return ContactModel
    if name in ('ContactAnalyzer', 'ContactBackend', 'MeshContactBackend'):
        from .contact import backends
        return getattr(backends, name)
    if name in ('SDFContactBackend', 'SDFConfig'):
        from .contact import sdf_backend
        return getattr(sdf_backend, name)
    if name in ('GridSDF', 'SignedDistanceField', 'SDFSamples'):
        from .geometry import sdf
        return getattr(sdf, name)
    if name in ('analyze_pair', 'analyze_contacts'):
        from .contact import analysis
        return getattr(analysis, name)
    raise AttributeError(name)
