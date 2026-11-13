"""Create complete inventory and SHA256SUMS only after all writers have ended."""
from pathlib import Path
import argparse,datetime,hashlib,json
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();r=a.root
for name in ['verification.json','video-validation.json','supplemental-gpu0-validation.json']:
 assert json.loads((r/'results'/name).read_text())['status']=='PASS'
assert (r/'README_zh.md').exists()
files=sorted(p for p in r.rglob('*') if p.is_file() and p.relative_to(r).as_posix() not in ['SHA256SUMS','results/file-manifest.json'])
assert not any('__pycache__' in p.parts for p in files)
records=[{'path':p.relative_to(r).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.file_digest(p.open('rb'),'sha256').hexdigest()} for p in files]
manifest={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'excludes':['SHA256SUMS','results/file-manifest.json'],'file_count':len(records),'total_bytes':sum(x['bytes'] for x in records),'files':records}
(r/'results/file-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
files=sorted(p for p in r.rglob('*') if p.is_file() and p.relative_to(r).as_posix()!='SHA256SUMS')
(r/'SHA256SUMS').write_text(''.join(f"{hashlib.file_digest(p.open('rb'),'sha256').hexdigest()}  {p.relative_to(r).as_posix()}\n" for p in files))
print(json.dumps({'files_hashed':len(files),'sha256sums_sha256':hashlib.sha256((r/'SHA256SUMS').read_bytes()).hexdigest(),'total_data_bytes':manifest['total_bytes']},indent=2))
