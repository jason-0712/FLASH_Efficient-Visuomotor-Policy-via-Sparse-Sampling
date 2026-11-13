"""Reverify reused10k raw package and frozen inputs on server, read-only."""
from pathlib import Path
import argparse,datetime,hashlib,json
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();r=a.root
checks=[]
for name,sha in [('heldout_eval_20261029','36a979fd37017a53019570e481d0a11e6ef9947ae9455b79b08f365511cc9b5e'),('heldout_initials_20261029','2724b400efb8878be1e772200fad2041521cfa6c4519de0b4d670c2f7d436904')]:
 p=r.parent/name;manifest=p/'SHA256SUMS';assert hashlib.sha256(manifest.read_bytes()).hexdigest()==sha
 n=0
 for line in manifest.read_text().splitlines():
  expected,fn=line.split('  ',1);assert hashlib.file_digest((p/fn).open('rb'),'sha256').hexdigest()==expected,(name,fn);n+=1
 checks.append({'package':name,'status':'PASS','files_verified':n,'sha256sums_sha256':sha})
x={'status':'PASS','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks};(r/'environment/reused-reference-integrity.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x,indent=2))
