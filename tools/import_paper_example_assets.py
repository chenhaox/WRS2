"""Copy only paper example STL and JSON inputs; never execute legacy code."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

MODELS = ('bigL','smallL_2','Z','T','cross','crossleft','3u','2u','u','1','2L_left','2L_right','domino')
SCENES = ('datainfo2','domino_5','burrpuzzle','bridge','datainfo7','datainfo0209_3','datainfo4',
          'domino_7','domino_8','domino_6','datainfo_ss','datainfo_q')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('legacy_root',type=Path)
    args = parser.parse_args(); root = args.legacy_root.resolve()
    source_commit = subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    out = Path(__file__).resolve().parents[1]/'examples/assembly/assets/paper2021'
    records = []
    for folder,names,suffix,target in (('objects',MODELS,'.stl','meshes'),('data',SCENES,'','scenes')):
        for name in names:
            src = root/'asp'/folder/(name+suffix)
            dst = out/target/(name+(suffix or '.json')); dst.parent.mkdir(parents=True,exist_ok=True)
            if folder=='data': json.loads(src.read_text(encoding='utf-8'))
            shutil.copyfile(src,dst)
            records.append(dict(source=src.relative_to(root).as_posix(),copy=dst.relative_to(out).as_posix(),
                                sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
    (out/'provenance.json').write_text(json.dumps(dict(
        source_repository='https://github.com/wrslab/assembly_planner',
        source_commit=source_commit,
        units='STL coordinates and scene translations interpreted as mm, declared explicitly',
        missing=['asp/objects/alframe.stl','asp/data/datainfo0209','asp/data/datainfo0209_2'],
        assets=records),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(f'Copied {len(records)} data files to {out}')


if __name__=='__main__': main()
