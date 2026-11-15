"""Fit/export a small area/aspect RBF model; NumPy-only deployment evaluation."""
from pathlib import Path
import json
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.model_selection import StratifiedGroupKFold
from compare_models import summary

ROOT=Path(__file__).resolve().parent


def export(scaler,model):
    return {'features':['log_area_fraction','rotated_box_width_over_height'],
        'classes':model.classes_.tolist(),'mean':scaler.mean_.tolist(),'scale':scaler.scale_.tolist(),
        'support_vectors':model.support_vectors_.tolist(),'dual_coef':model.dual_coef_.tolist(),
        'intercept':model.intercept_.tolist(),'n_support':model.n_support_.tolist(),'gamma':float(model._gamma)}


def evaluate(data,x):
    x=np.atleast_2d(x)
    z=(x-np.asarray(data['mean']))/np.asarray(data['scale'])
    sv=np.asarray(data['support_vectors']);dual=np.asarray(data['dual_coef'])
    kernel=np.exp(-data['gamma']*np.sum((z[:,None,:]-sv[None,:,:])**2,axis=2))
    n=data['n_support'];start=np.r_[0,np.cumsum(n)]
    votes=np.zeros((len(z),3),int);pair_scores=[];pairs=[];k=0
    for i in range(3):
        for j in range(i+1,3):
            score=kernel[:,start[i]:start[i+1]]@dual[j-1,start[i]:start[i+1]]
            score+=kernel[:,start[j]:start[j+1]]@dual[i,start[j]:start[j+1]]
            score+=data['intercept'][k];k+=1
            votes[:,i]+=(score>0);votes[:,j]+=(score<=0)
            pair_scores.append(score);pairs.append((i,j))
    winner=np.argmax(votes,axis=1)
    margins=[]
    for row,win in enumerate(winner):
        margins.append(min(abs(pair_scores[k][row]) for k,pair in enumerate(pairs) if win in pair)
                       if votes[row,win]==2 else 0.)
    return np.asarray(data['classes'])[winner],np.asarray(margins),np.stack(pair_scores,axis=1)


def main():
    data=json.loads((ROOT/'measurements.json').read_text());rows=[d['row'] for d in data]
    x=np.array([[r['log_area'],r['box_ratio']] for r in rows]);y=np.array([r['label'] for r in rows])
    groups=np.array([r['label']+'_'+str(r['index']//3) for r in rows])
    all_pred=[];all_margins=[];diagnostics=[]
    for seed in [0,13,37]:
        pred=np.empty(len(y),dtype='<U5');margin=np.zeros(len(y))
        for train,test in StratifiedGroupKFold(5,shuffle=True,random_state=seed).split(x,y,groups):
            scaler=StandardScaler().fit(x[train]);model=SVC(C=10,gamma=1,decision_function_shape='ovo').fit(scaler.transform(x[train]),y[train])
            exported=export(scaler,model);pp,mm,score=evaluate(exported,x[test])
            assert np.array_equal(pp,model.predict(scaler.transform(x[test])))
            np.testing.assert_allclose(score,model.decision_function(scaler.transform(x[test])),atol=1e-10)
            pred[test]=pp;margin[test]=mm
        all_pred.append(pred);all_margins.append(margin)
        for i in np.flatnonzero(pred!=y):
            diagnostics.append({'seed':seed,'label':y[i],'prediction':pred[i],
                'margin':float(margin[i]),**rows[i]})
    yy=np.tile(y,3);pp=np.concatenate(all_pred);mm=np.concatenate(all_margins)
    rejection=[]
    for cutoff in [0,.05,.10,.15,.20,.30]:
        accepted=mm>=cutoff
        conversion=accepted&(pp!='stand')
        rejection.append({'cutoff':cutoff,'coverage':float(np.mean(accepted)),
            'accepted_accuracy':float(np.mean(pp[accepted]==yy[accepted])),
            'standard_false_conversion':float(np.mean(conversion[yy=='stand'])),
            'special_missed_conversion':float(np.mean(~conversion[yy!='stand']))})
    scaler=StandardScaler().fit(x);model=SVC(C=10,gamma=1,decision_function_shape='ovo').fit(scaler.transform(x),y)
    exported=export(scaler,model)
    # Verify exact exported implementation over the entire useful feature plane.
    rng=np.random.default_rng(5)
    grid=np.column_stack([rng.uniform(x[:,0].min()-.3,x[:,0].max()+.3,10000),rng.uniform(.15,1.5,10000)])
    gp,_,gs=evaluate(exported,grid)
    assert np.array_equal(gp,model.predict(scaler.transform(grid)))
    np.testing.assert_allclose(gs,model.decision_function(scaler.transform(grid)),atol=1e-10)
    exported.update({'training_samples':len(y),'training_area_fraction_range':[float(np.exp(x[:,0].min())),float(np.exp(x[:,0].max()))],
                     'training_box_ratio_range':[float(x[:,1].min()),float(x[:,1].max())],
                     'default_margin':.15,'dataset_class_counts':{c:int(np.sum(y==c)) for c in model.classes_}})
    (ROOT/'area_ratio_model.json').write_text(json.dumps(exported,indent=2),encoding='utf-8')
    report={'grouped_summary':summary(yy,pp),'rejection_tradeoff':rejection,'errors':diagnostics,
            'training_fit':summary(y,model.predict(scaler.transform(x))),
            'support_vectors':len(model.support_vectors_),'export_verified_random_points':10000}
    (ROOT/'validation_details.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
