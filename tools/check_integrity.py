"""Verify every published payload, including hydrated Git LFS files."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'checksums.json').read_text())
errors=[]
for name,expected in manifest['files'].items():
    path=(root/name).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        errors.append(name+': missing/unsafe')
    elif hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        errors.append(name+': checksum mismatch (run git lfs pull if needed)')
if errors:
    raise SystemExit('\n'.join(errors))
print(f"Verified {len(manifest['files'])} published files")
