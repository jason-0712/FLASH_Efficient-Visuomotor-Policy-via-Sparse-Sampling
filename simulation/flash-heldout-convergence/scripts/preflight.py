"""Read-only CPU verification of declared inputs and source before evaluation."""
import argparse,datetime,hashlib,json,platform,subprocess
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--frozen-root',type=Path,required=True);ap.add_argument('--repo',type=Path,required=True)
a=ap.parse_args();p=json.loads((a.root/'configs/frozen-evaluation-plan.json').read_text());sha=lambda f:hashlib.file_digest(f.open('rb'),'sha256').hexdigest()
records=[]
for e in p['checkpoints']:
 f=Path(e['checkpoint_path']);h=sha(f);assert h==e['checkpoint_sha256'];records.append({'policy':e['policy'],'step':e['checkpoint_step'],'sha256':h,'bytes':f.stat().st_size})
assert sha(a.frozen_root/p['test_initials_relative_path'])==p['test_initials_sha256']
assert sha(a.root/p['loader_relative_path'])==p['loader_sha256']
commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=a.repo,text=True).strip();diff=hashlib.sha256(subprocess.check_output(['git','diff','--binary'],cwd=a.repo)).hexdigest()
assert commit=='f47e490f2b820f835d1b9145da3cdb3df69f50a0';assert diff=='56fec88a58c6475de026bc2a4bf81fb4a797679c6a7ff104676149accfb677d9'
r={'status':'PASS','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'hostname':platform.node(),'checkpoints':records,'frozen_initials_and_loader_match':True,'commit':commit,'diff_sha256':diff}
(a.root/'environment/preflight.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
