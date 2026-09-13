"""M2 works in a process that explicitly forbids robot/physics/GPU imports."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from wrs.assembly import save_assembly
from wrs.assembly.geometry.primitives import box,pose
from wrs.assembly import Assembly,Part


class EndToEndTests(unittest.TestCase):
    def test_core_import_boundary_and_real_plan(self):
        script='''
import builtins
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if any(name==p or name.startswith(p+'.') for p in ('mujoco','open3d','wgpu','wrs.robots','wrs.viewer','wrs.grasp')):
        raise RuntimeError('Forbidden optional import: '+name)
    return original(name,*args,**kwargs)
builtins.__import__=guarded
from wrs.assembly import Assembly,Part,plan_sequence,replay_sequence
from wrs.assembly.geometry.primitives import box,pose
a=Assembly((Part('base',box((.2,.2,.02)),pose((0,0,-.01)),fixed=True,friction=.5),
            Part('part',box((.02,.02,.02)),pose((0,0,.01)),mass_kg=.1,com_local_m=(0,0,0),friction=.5)))
p=plan_sequence(a)
assert p.status=='success' and replay_sequence(a,p)['status']=='valid'
assert not p.execution_validated
print('M2 headless boundary passed')
'''
        r=subprocess.run([sys.executable,'-c',script],cwd=Path(__file__).resolve().parents[2],capture_output=True,text=True,timeout=30)
        self.assertEqual(r.returncode,0,r.stderr)

    def test_manifest_cli_and_versioned_report(self):
        import json
        a=Assembly((Part('base',box((.2,.2,.02)),pose((0,0,-.01)),fixed=True,friction=.5),
                    Part('part',box((.02,.02,.02)),pose((0,0,.01)),mass_kg=.1,com_local_m=(0,0,0),friction=.5)))
        with tempfile.TemporaryDirectory() as folder:
            manifest=Path(folder)/'input.json'; output=Path(folder)/'plan.json'; save_assembly(a,manifest)
            r=subprocess.run([sys.executable,'tools/assembly/plan_manifest.py',str(manifest),'--output',str(output)],
                             cwd=Path(__file__).resolve().parents[2],capture_output=True,text=True,timeout=30)
            self.assertEqual(r.returncode,0,r.stderr)
            report=json.loads(output.read_text(encoding='utf-8'))
            self.assertEqual(report['schema_version'],'wrs.assembly.sequence/1')
            self.assertEqual(report['status'],'success')
            self.assertFalse(report['execution_validated'])


if __name__=='__main__': unittest.main()
