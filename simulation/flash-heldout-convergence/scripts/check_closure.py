"""Record post-run source integrity, process release, and successful CPU outputs."""
import argparse,datetime,hashlib,json,os,subprocess
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();r=a.root;repo=r.parent/'FLASH-Policy'
for name in ['verification.json','video-validation.json','normalizer-audit.json','supplemental-gpu0-validation.json','bridge-validation.json']:
 assert json.loads((r/'results'/name).read_text())['status']=='PASS',name
for name in ['gpu0','gpu2','gpu2-2500']:assert json.loads((r/f'results/job-{name}-exit.json').read_text())['exit_code']==0
commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip();diff=hashlib.sha256(subprocess.check_output(['git','diff','--binary'],cwd=repo)).hexdigest();assert commit=='f47e490f2b820f835d1b9145da3cdb3df69f50a0';assert diff=='56fec88a58c6475de026bc2a4bf81fb4a797679c6a7ff104676149accfb677d9'
wanted={str(r/'scripts'/s) for s in ['evaluate_heldout.py','run_worker.sh','run_gpu2_2500.sh','finish_cpu.sh']};remaining=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  argv=p.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
  if wanted.intersection(argv):remaining.append({'pid':int(p.name),'argv':argv})
 except (OSError,ProcessLookupError):pass
assert not remaining,remaining
for query,name in [('index,uuid,name,memory.used,utilization.gpu','gpus-final.csv')]:
 (r/'environment'/name).write_text(subprocess.check_output(['nvidia-smi',f'--query-gpu={query}','--format=csv'],text=True))
(r/'environment/processes-final.csv').write_text(subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory','--format=csv'],text=True))
(r/'environment/git-status-final.txt').write_text(subprocess.check_output(['git','status','--short'],cwd=repo,text=True))
monitors=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  argv=p.joinpath('cmdline').read_bytes().decode(errors='replace').split('\0')
  if argv and Path(argv[0]).name=='nvidia-smi' and '-l' in argv:monitors.append({'pid':int(p.name),'argv':argv})
 except OSError:pass
result={'status':'PASS','verified_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'all_formal_and_supplemental_cpu_checks_passed':True,'source_commit':commit,'source_diff_sha256':diff,'own_evaluation_or_worker_processes_remaining':remaining,'visible_periodic_nvidia_smi_processes':monitors,'gpu0_stop_check_found_no_own_process_because_job_already_ended':True}
(r/'environment/closure.json').write_text(json.dumps(result,indent=2)+'\n');(r/'results/cpu-analysis-complete.json').write_text(json.dumps({'status':'PASS','finished_utc':result['verified_utc'],'first_aggregate_assertion_corrected_without_reevaluation':True},indent=2)+'\n');print(json.dumps(result,indent=2))
