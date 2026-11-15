"""Grouped model comparison/export on the six newly captured folders."""
from pathlib import Path
import sys,json
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.model_selection import StratifiedGroupKFold
from sklearn.metrics import confusion_matrix
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'pillar_area_model'))
from build_model import export,evaluate
FEATURES={'area_box':['log_area','box_ratio'],'area_avg':['log_area','avg_ratio'],
 'area_both':['log_area','box_ratio','avg_ratio'],'area_full_box':['log_area','full_box_ratio'],
 'area_full_avg':['log_area','full_avg_ratio'],'area_full_both':['log_area','full_box_ratio','full_avg_ratio'],
 'ratio_only':['box_ratio','avg_ratio']}
LABELS=['tall','stand','low']
def summary(y,p,margin=None,cutoff=0):
 accepted=np.ones(len(y),bool) if margin is None else margin>=cutoff
 convert=accepted & (p!='stand')
 return {'n':len(y),'accuracy':float(np.mean(y==p)),'coverage':float(accepted.mean()),
   'accepted_accuracy':float(np.mean(y[accepted]==p[accepted])) if accepted.any() else None,
   'standard_false_conversion':float(convert[y=='stand'].mean()),
   'special_missed_conversion':float((~convert[y!='stand']).mean()),
   'confusion':confusion_matrix(y,p,labels=LABELS).tolist()}
def main():
 rows=json.loads((ROOT/'measurements.json').read_text());y=np.array([r['label'] for r in rows]);colors=np.array([r['color'] for r in rows])
 # Adjacent frames are grouped; these are not independent camera sessions.
 groups=np.array([r['folder']+'_'+str(r['index']//4) for r in rows])
 all_reports=[];selected={};rng=np.random.default_rng(7)
 for color in ['red','green']:
  target=colors==color;best=None
  cache=json.loads((ROOT/'model_comparison.json').read_text()) if (ROOT/'model_comparison.json').exists() else []
  chosen=min([r for r in cache if r['color']==color],key=lambda r:(r['selection_loss'],len(r['features']),r['C'])) if cache else None
  for scope in ['pooled','color_specific']:
   selection=np.ones(len(y),bool) if scope=='pooled' else target
   ix=np.flatnonzero(selection);ys=y[ix];gs=groups[ix]
   folds=[]
   for seed in [0,13,37]:
    folds.append(list(StratifiedGroupKFold(5,shuffle=True,random_state=seed).split(ix,ys,gs)))
   for fs,keys in FEATURES.items():
    x=np.array([[r[k] for k in keys] for r in rows])[ix]
    for c,gamma in [(1,1),(10,.3),(10,1),(10,3),(100,.3),(100,1)]:
     if chosen and (scope,fs,c,gamma)!=(chosen['scope'],chosen['feature_set'],chosen['C'],chosen['gamma']):continue
     predictions=[];margins=[]
     for ff in folds:
      p=np.empty(len(ix),dtype='<U5');margin=np.zeros(len(ix))
      for train,test in ff:
       scaler=StandardScaler().fit(x[train]);model=SVC(C=c,gamma=gamma,decision_function_shape='ovo').fit(scaler.transform(x[train]),ys[train])
       e=export(scaler,model);pp,mm,score=evaluate(e,x[test]);p[test]=pp;margin[test]=mm
      keep=colors[ix]==color;predictions.append(p[keep]);margins.append(margin[keep])
     truth=np.tile(y[target],3);pred=np.concatenate(predictions);mar=np.concatenate(margins)
     result={'color':color,'scope':scope,'feature_set':fs,'features':keys,'C':c,'gamma':gamma,**summary(truth,pred)}
     # Penalize repainting a standard pillar, but also account for missed specials.
     loss=2*result['standard_false_conversion']+result['special_missed_conversion']+.3*(1-result['accuracy'])
     result['selection_loss']=loss;all_reports.append(result)
     rank=(loss,len(keys),c)
     if best is None or rank<best[0]:best=(rank,result,pred,mar,ix,x)
  _,r,pred,margin,ix,x=best
  cutoff_reports=[{'cutoff':cut,**summary(np.tile(y[target],3),pred,margin,cut)} for cut in [0,.05,.10,.15,.2,.3,.5]]
  # Keep a conservative uncertainty gate; it is not a calibrated probability.
  cutoff=.15
  scaler=StandardScaler().fit(x);model=SVC(C=r['C'],gamma=r['gamma'],decision_function_shape='ovo').fit(scaler.transform(x),y[ix])
  exported=export(scaler,model);exported['features']=r['features'];exported['area_range']=[min(rows[i]['model_area_fraction'] for i in ix),max(rows[i]['model_area_fraction'] for i in ix)]
  exported['training_samples']=len(ix);exported['selection']=r;exported['default_margin']=cutoff
  xp=np.array([[rr[k] for k in r['features']] for rr in rows])[target]
  pp,mm,score=evaluate(exported,xp);np.testing.assert_allclose(score,model.decision_function(scaler.transform(xp)),atol=1e-10)
  grid=rng.uniform(x.min(axis=0)-.1,x.max(axis=0)+.1,(3000,x.shape[1]));gp,gm,gs=evaluate(exported,grid)
  assert np.array_equal(gp,model.predict(scaler.transform(grid)))
  # Chronological first-half -> second-half and reverse stress check.
  first=np.array([rr['index']<sum(qq['folder']==rr['folder'] for qq in rows)/2 for rr in rows])[ix]
  cp=np.empty(len(ix),dtype='<U5')
  for train in [first,~first]:
   sc=StandardScaler().fit(x[train]);mo=SVC(C=r['C'],gamma=r['gamma']).fit(sc.transform(x[train]),y[ix][train]);cp[~train]=mo.predict(sc.transform(x[~train]))
  selected[color]={'model':exported,'cutoff_comparison':cutoff_reports,'grouped':summary(np.tile(y[target],3),pred,margin,cutoff),
    'training_replay':summary(y[target],pp,mm,cutoff), 'chronological_half_stress':summary(y[ix][colors[ix]==color],cp[colors[ix]==color]),
    'errors':[{'seed_index':int(i//target.sum()),**rows[np.flatnonzero(target)[i%target.sum()]],'prediction':pred[i],'margin':float(margin[i])} for i in np.flatnonzero(pred!=np.tile(y[target],3))]}
  print(color,json.dumps({'selected':r,'gated':selected[color]['grouped'],'replay':selected[color]['training_replay'],'chronological':selected[color]['chronological_half_stress']}),flush=True)
 all_reports.sort(key=lambda r:(r['color'],r['selection_loss']))
 if len(all_reports)==2:all_reports=json.loads((ROOT/'model_comparison.json').read_text())
 (ROOT/'model_comparison.json').write_text(json.dumps(all_reports,indent=2),encoding='utf-8')
 (ROOT/'models.json').write_text(json.dumps({c:v['model'] for c,v in selected.items()},indent=2),encoding='utf-8')
 (ROOT/'validation_report.json').write_text(json.dumps(selected,indent=2),encoding='utf-8')
if __name__=='__main__':main()
