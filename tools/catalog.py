"""Dependency-free catalog lookup; simulation engines are loaded by the caller."""
from pathlib import Path
import argparse
import json

ROOT = Path(__file__).resolve().parents[1]

def list_assets(origin=None):
    index=json.loads((ROOT/'catalog/index.json').read_text())
    assets=[json.loads((ROOT/a['manifest']).read_text()) for a in index['assets']]
    return [a for a in assets if origin is None or a['origin']==origin]

def resolve(asset_id, backend):
    for a in list_assets():
        if a['id']==asset_id:
            path=(ROOT/a['representations'][backend]).resolve()
            if not path.is_relative_to(ROOT) or not path.is_file():
                raise ValueError('Missing or external asset entry')
            return path
    raise KeyError(asset_id)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--origin',choices=['fq','simple'])
    p.add_argument('--resolve')
    p.add_argument('--backend',choices=['mujoco','usd'],default='mujoco')
    args=p.parse_args()
    if args.resolve:
        print(resolve(args.resolve,args.backend))
    else:
        for a in list_assets(args.origin):
            print(a['id'],a['version'],a['kind'],a['origin'],a['upstream_origin'],sep='\t')
