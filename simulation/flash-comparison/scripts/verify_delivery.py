"""Verify a copied delivery using file hashes, without reading checkpoints into memory."""
import argparse,datetime,hashlib,json,socket
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--create-manifest',action='store_true');p.add_argument('--write-receipt',action='store_true');p.add_argument('--sha256sums',action='store_true');a=p.parse_args()
root=Path(a.root).resolve();manifest=root/'manifests/delivery-files.json'
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
if a.create_manifest:
    excluded={'SHA256SUMS','manifests/delivery-files.json','results/remote-delivery-verification.json'}
    rows=[{'path':str(p.relative_to(root)),'bytes':p.stat().st_size,'sha256':digest(p)} for p in sorted(root.rglob('*')) if p.is_file() and str(p.relative_to(root)) not in excluded and '__pycache__' not in p.parts]
    manifest.write_text(json.dumps({'files':rows,'count':len(rows),'bytes':sum(r['bytes'] for r in rows)},indent=2,ensure_ascii=False))
    print(json.dumps({'manifest_created':str(manifest),'count':len(rows),'bytes':sum(r['bytes'] for r in rows)}));raise SystemExit
if a.sha256sums:
    rows=[]
    for line in (root/'SHA256SUMS').read_text().splitlines():
        sha,path=line.split('  ',1);rows.append({'path':path,'sha256':sha})
else:rows=json.loads(manifest.read_text())['files']
total=0
for row in rows:
    path=(root/row['path']).resolve();assert path.is_relative_to(root)
    assert path.is_file(),path
    assert digest(path)==row['sha256'],path
    if 'bytes' in row:assert path.stat().st_size==row['bytes'],path
    total+=path.stat().st_size
result={'verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'host':socket.gethostname(),'files':len(rows),'bytes':total,'all_hashes_match':True,'manifest_sha256':digest(root/'SHA256SUMS' if a.sha256sums else manifest)}
if a.write_receipt:(root/'results/remote-delivery-verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
