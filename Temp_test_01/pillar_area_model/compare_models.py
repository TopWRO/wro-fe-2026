"""Exploratory grouped validation: joint area/aspect models vs fixed thresholds."""
from pathlib import Path
import json
import numpy as np
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler,PolynomialFeatures
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import StratifiedGroupKFold
from sklearn.metrics import confusion_matrix

ROOT=Path(__file__).resolve().parent
LABELS=['tall','stand','low']


def candidates():
    return {
        'linear':lambda:make_pipeline(StandardScaler(),LogisticRegression(C=10,max_iter=3000)),
        'poly2_C1':lambda:make_pipeline(StandardScaler(),PolynomialFeatures(2,include_bias=False),LogisticRegression(C=1,max_iter=3000)),
        'poly2_C10':lambda:make_pipeline(StandardScaler(),PolynomialFeatures(2,include_bias=False),LogisticRegression(C=10,max_iter=3000)),
        'poly3_C1':lambda:make_pipeline(StandardScaler(),PolynomialFeatures(3,include_bias=False),LogisticRegression(C=1,max_iter=3000)),
        'rbf_C10':lambda:make_pipeline(StandardScaler(),SVC(C=10,gamma=1)),
        'knn3':lambda:make_pipeline(StandardScaler(),KNeighborsClassifier(3,weights='distance')),
        'knn5':lambda:make_pipeline(StandardScaler(),KNeighborsClassifier(5,weights='distance')),
        'tree3':lambda:DecisionTreeClassifier(max_depth=3,min_samples_leaf=3,random_state=0),
        'tree4':lambda:DecisionTreeClassifier(max_depth=4,min_samples_leaf=3,random_state=0),
    }


def summary(truth,pred):
    cm=confusion_matrix(truth,pred,labels=LABELS)
    return {'accuracy':float(np.mean(truth==pred)),
            'standard_false_conversion':float(np.mean(pred[truth=='stand']!='stand')),
            'special_missed_conversion':float(np.mean(pred[truth!='stand']=='stand')),
            'confusion':cm.tolist()}


def main():
    data=json.loads((ROOT/'measurements.json').read_text())
    rows=[d['row'] for d in data]
    y=np.array([r['label'] for r in rows])
    groups=np.array([r['label']+'_'+str(r['index']//3) for r in rows])
    feature_sets={'avg2':['log_area','avg_ratio'],'box2':['log_area','box_ratio'],
                  'both3':['log_area','avg_ratio','box_ratio']}
    results=[]
    for fs,features in feature_sets.items():
        x=np.array([[r[k] for k in features] for r in rows])
        for name,factory in candidates().items():
            predictions=[];scores=[]
            for seed in [0,13,37]:
                pred=np.empty(len(y),dtype='<U5')
                for train,test in StratifiedGroupKFold(5,shuffle=True,random_state=seed).split(x,y,groups):
                    model=factory();model.fit(x[train],y[train]);pred[test]=model.predict(x[test])
                predictions.append(pred.tolist());scores.append(summary(y,pred))
            # Different approach/recede samples: divide each class at its area peak.
            peak={c:max((r for r in rows if r['label']==c),key=lambda r:r['area_pixels'])['index'] for c in LABELS}
            first=np.array([r['index']<=peak[r['label']] for r in rows])
            cross_pred=np.empty(len(y),dtype='<U5')
            for training in [first,~first]:
                model=factory();model.fit(x[training],y[training]);cross_pred[~training]=model.predict(x[~training])
            result={'features':fs,'model':name,'grouped_mean_accuracy':float(np.mean([s['accuracy'] for s in scores])),
                    'grouped_standard_false_conversion':float(np.mean([s['standard_false_conversion'] for s in scores])),
                    'grouped_special_missed_conversion':float(np.mean([s['special_missed_conversion'] for s in scores])),
                    'grouped_scores':scores,'grouped_predictions':predictions,
                    'peak_split':summary(y,cross_pred),'peak_split_predictions':cross_pred.tolist()}
            results.append(result)
    results.sort(key=lambda r:(-r['grouped_mean_accuracy'],r['grouped_standard_false_conversion']))
    baselines={}
    for key in ['avg_ratio','box_ratio']:
        ratio=np.array([r[key] for r in rows])
        for lo,hi in [(.30,.60),(.44,.60)]:
            pred=np.where(ratio<lo,'tall',np.where(ratio>hi,'low','stand'))
            baselines[f'{key}:{lo}-{hi}']=summary(y,pred)
    (ROOT/'model_comparison.json').write_text(json.dumps({'label_order':LABELS,'n':len(y),
        'group_definition':'Three consecutive frames within each labeled folder; no independent capture sessions available',
        'results':results,'baselines':baselines},indent=2),encoding='utf-8')
    for r in results:
        print(r['features'],r['model'],'CV',round(r['grouped_mean_accuracy'],3),
              'stdwrong',round(r['grouped_standard_false_conversion'],3),'peak',round(r['peak_split']['accuracy'],3))
    print('BASELINES',baselines)


if __name__=='__main__':main()
