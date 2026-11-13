"""Verify previous finalized packages without mutating them."""
from pathlib import Path
import argparse,datetime,hashlib,json
ap=argparse.ArgumentParser();ap.add_argument('--outputs',type=Path,required=True);ap.add_argument('--destination',type=Path,required=True);a=ap.parse_args();records=[]
for name,expected in [('flash-reproduction','a9d954bd8d8b7cd4c0b6aecdfb2a16126dcbc9751e1dbce17968606513494c5c'),('flash-comparison','51522d1c77a28e338753f1ba8ffa7566280db8691c531f1a4af0c6759457b1c9'),('flash-placement-audit','fd773c490b7159f6a6142afa898c02d71718decdd828489180f8fe20f5f1cb79'),('flash-heldout-initials','2724b400efb8878be1e772200fad2041521cfa6c4519de0b4d670c2f7d436904'),('flash-heldout-evaluation','36a979fd37017a53019570e481d0a11e6ef9947ae9455b79b08f365511cc9b5e')]:
 root=a.outputs/name;manifest=root/'SHA256SUMS';assert hashlib.sha256(manifest.read_bytes()).hexdigest()==expected
 n=0
 for line in manifest.read_text().splitlines():
  sha,fn=line.split('  ',1);f=root/fn;assert hashlib.file_digest(f.open('rb'),'sha256').hexdigest()==sha,(name,fn);n+=1
 records.append({'package':name,'status':'PASS','files_verified':n,'sha256sums_sha256':expected})
result={'status':'PASS','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'packages':records};a.destination.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
