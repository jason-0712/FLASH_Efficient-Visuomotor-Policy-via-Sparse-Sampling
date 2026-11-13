"""Verify every checksum and require complete file coverage, read-only."""
from pathlib import Path
import argparse,hashlib,json
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();r=a.root;records={}
for line in (r/'SHA256SUMS').read_text().splitlines():
 sha,fn=line.split('  ',1);assert fn not in records;records[fn]=sha
 p=r/fn;assert hashlib.file_digest(p.open('rb'),'sha256').hexdigest()==sha,fn
actual={p.relative_to(r).as_posix() for p in r.rglob('*') if p.is_file() and p.name!='SHA256SUMS'}
assert set(records)==actual,(set(records)-actual,actual-set(records))
m=json.loads((r/'results/file-manifest.json').read_text());assert m['file_count']==len(m['files'])
assert {x['path'] for x in m['files']}==actual-{'results/file-manifest.json'}
for x in m['files']:
 assert records[x['path']]==x['sha256'] and (r/x['path']).stat().st_size==x['bytes']
print(json.dumps({'status':'PASS','files_verified':len(records),'complete_coverage':True,'sha256sums_sha256':hashlib.sha256((r/'SHA256SUMS').read_bytes()).hexdigest()},indent=2))
