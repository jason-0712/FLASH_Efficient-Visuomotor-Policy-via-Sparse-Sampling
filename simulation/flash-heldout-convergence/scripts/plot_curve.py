"""Export standalone scientific curve and paired-outcome raster."""
import argparse,csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
import numpy as np
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();root=a.root
s=json.loads((root/'results/convergence.json').read_text());points=s['curve_points'];out=root/'plots';out.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none','pdf.fonttype':42})
fig,ax=plt.subplots(figsize=(9.5,6.0));fig.subplots_adjust(left=.10,right=.96,top=.84,bottom=.22)
x=np.array([p['checkpoint_step'] for p in points])
for policy,label,color,offset in [('flash','FLASH (ds4, 1 inference step)','#1a716a',-100),('fm_dit','FM-DiT (ds1, 10 inference steps)','#b05a22',100)]:
 y=np.array([p['policies'][policy]['success_rate']*100 for p in points]);ci=np.array([p['policies'][policy]['descriptive_wilson_95'] for p in points])*100
 ax.fill_between(x,ci[:,0],ci[:,1],color=color,alpha=.10)
 ax.plot(x,y,color=color,marker='o' if policy=='flash' else 's',lw=2,ms=7,label=label)
 ax.errorbar(x,y,yerr=np.maximum(0,np.vstack([y-ci[:,0],ci[:,1]-y])),fmt='none',color=color,alpha=.55,capsize=4,lw=1)
 for xx,yy,p in zip(x,y,points):
  k=p['policies'][policy]['successes'];ax.annotate(f'{k}/50',(xx,yy),xytext=(0,11 if policy=='flash' else -21),textcoords='offset points',ha='center',color=color,fontweight='bold',fontsize=10)
ax.set(xticks=x,xticklabels=[f'{v:,}' for v in x],yticks=np.arange(0,101,20),ylim=(-3,112),xlim=(2200,10300),xlabel='Training updates (existing checkpoints)',ylabel='Closed-loop SuccessOnce (%)')
ax.grid(axis='y',alpha=.18);ax.legend(loc='lower right',frameon=False,fontsize=10)
fig.text(.10,.94,'Unseen object-position convergence',fontsize=18,fontweight='bold')
fig.text(.10,.885,'Pick-Place Bowl | same frozen 50 initial states | training seed 42',fontsize=11,color='#454545')
fig.text(.10,.12,'All four checkpoints: physical GPU 2; 10,000: reused prior evaluation.',fontsize=9,color='#555555')
fig.text(.10,.085,'Bands / bars: descriptive 95% Wilson intervals; checkpoints share the same 50 cases.',fontsize=9,color='#555555')
fig.text(.10,.05,'Original geometric success with first-success stop; release and stable placement are not scored.',fontsize=9,color='#555555')
for ext in ['png','svg','pdf']:fig.savefig(out/f'heldout-convergence.{ext}',dpi=200)
plt.close(fig)
with (root/'results/paired-outcomes.csv').open() as f:rows=list(csv.DictReader(f))
steps=[2500,5000,7500,10000];categories=['both_fail','fm_dit_only_success','flash_only_success','both_success'];colors=['#d7dbe0','#d4884e','#68b5a3','#304d6b']
a=np.zeros((4,50),int)
for r in rows:a[steps.index(int(r['checkpoint_step'])),int(r['test_id'])]=categories.index(r['category'])
fig,ax=plt.subplots(figsize=(12,4.0));fig.subplots_adjust(left=.10,right=.98,top=.78,bottom=.30)
ax.imshow(a,aspect='auto',cmap=ListedColormap(colors),norm=BoundaryNorm(np.arange(-.5,4.5),4),interpolation='nearest')
ax.set(yticks=range(4),yticklabels=[f'{x:,}' for x in steps],xticks=np.arange(0,50,5),xlabel='Frozen test ID',ylabel='Training updates')
ax.set_xticks(np.arange(-.5,50,1),minor=True);ax.set_yticks(np.arange(-.5,4,1),minor=True);ax.grid(which='minor',color='white',lw=.5);ax.tick_params(which='minor',bottom=False,left=False)
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color=c,label=l) for c,l in zip(colors,['Both fail','FM-DiT only','FLASH only','Both succeed'])],loc='upper center',bbox_to_anchor=(.5,-.34),ncol=4,frameon=False,fontsize=10)
fig.text(.10,.92,'Paired outcomes on the same 50 initial states',fontsize=17,fontweight='bold')
fig.text(.10,.84,'One cell = one frozen initial state at one checkpoint; no cases removed.',fontsize=10,color='#555555')
for ext in ['png','svg','pdf']:fig.savefig(out/f'paired-outcomes.{ext}',dpi=200)
plt.close(fig)
print('PLOTS_EXPORTED',out)
