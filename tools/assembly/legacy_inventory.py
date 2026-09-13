"""Inspect old asp assets and convert complete assemblies, without legacy imports."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from wrs.assembly import save_report, save_assembly
from wrs.assembly.adapters.legacy import legacy_asset_inventory, load_legacy_assembly


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-root',type=Path,required=True)
    parser.add_argument('--length-unit',choices=['m','mm','cm'],required=True)
    parser.add_argument('--names',nargs='+',default=['domino_5','burrpuzzle','bridge'])
    parser.add_argument('--out-dir',type=Path,default=ROOT/'benchmark_results'/'assembly'/'legacy')
    args = parser.parse_args()
    commit = subprocess.run(['git','-C',str(args.legacy_root),'rev-parse','HEAD'],capture_output=True,text=True)
    reports = []
    for name in args.names:
        report = legacy_asset_inventory(args.legacy_root,name,length_unit=args.length_unit)
        report['source_commit'] = commit.stdout.strip() if commit.returncode==0 else None
        missing = sorted({p['path'] for p in report['parts'] if not p['exists']})
        report['missing_assets'] = missing
        if not missing:
            assembly = load_legacy_assembly(args.legacy_root,name,length_unit=args.length_unit)
            save_assembly(assembly,args.out_dir/f'{name}.assembly.json')
        reports.append(report)
        print(f'{name}: {report["part_count"]} parts; {len(missing)} missing unique assets',flush=True)
    save_report(reports,args.out_dir/'inventory.json')
    print(args.out_dir/'inventory.json')


if __name__ == '__main__':
    main()
