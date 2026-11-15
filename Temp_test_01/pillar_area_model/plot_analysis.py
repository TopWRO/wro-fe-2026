from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from build_model import evaluate

ROOT=Path(__file__).resolve().parent
plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans']
plt.rcParams['axes.unicode_minus']=False
rows=[d['row'] for d in json.loads((ROOT/'measurements.json').read_text())]
model=json.loads((ROOT/'area_ratio_model.json').read_text())
names={'tall':'高柱 15 cm（33 张）','stand':'标准柱 10 cm（38 张）','low':'矮柱 5 cm（31 张）'}
colors={'tall':'#2761a8','stand':'#c77714','low':'#23805e'}
fig,axes=plt.subplots(1,2,figsize=(13,5.8),constrained_layout=True)
area_axis=np.geomspace(400,75000,350);ratio_axis=np.linspace(.15,1.30,260)
aa,rr=np.meshgrid(area_axis,ratio_axis)
pred,_,_=evaluate(model,np.column_stack([np.log(aa.ravel()/(640*480)),rr.ravel()]))
index=np.array([{'tall':0,'stand':1,'low':2}[s] for s in pred]).reshape(aa.shape)
axes[1].pcolormesh(aa,rr,index,cmap=ListedColormap(['#e5eef9','#fff0d9','#e3f3e9']),shading='auto',vmin=0,vmax=2,rasterized=True)
for ax,key,title in zip(axes,['avg_ratio','box_ratio'],['原平均边长比：类别仍有重叠','面积＋旋转框比：学到的分类区域']):
    for label in ['tall','stand','low']:
        subset=[r for r in rows if r['label']==label]
        ax.scatter([r['area_pixels'] for r in subset],[r[key] for r in subset],
                   c=colors[label],s=35,label=names[label],edgecolors='white',linewidths=.4,zorder=3)
        clipped=[r for r in subset if r['partial']]
        ax.scatter([r['area_pixels'] for r in clipped],[r[key] for r in clipped],
                   facecolors='none',edgecolors='black',s=85,linewidths=.8,zorder=4)
    ax.set_xscale('log');ax.set_xlim(400,75000);ax.set_ylim(.15,1.30)
    ax.set_xlabel('目标轮廓填充面积（像素，640×480；横轴为对数）')
    ax.set_ylabel('图像宽÷高');ax.set_title(title);ax.grid(alpha=.2)
axes[0].axhline(.30,color='#777777',ls='--',lw=1);axes[0].axhline(.60,color='#777777',ls='--',lw=1)
axes[0].legend(fontsize=9,loc='upper left')
fig.suptitle('102 张实拍：联合面积特征，比固定宽高比更适合本次数据',fontsize=15)
fig.supxlabel('右图背景来自全样本拟合；黑圈为触边样本。约98%的结果来自分段留出验证，非独立实车准确率。',fontsize=9)
fig.savefig(ROOT/'area_ratio_analysis.png',dpi=180)
plt.close(fig)
print('Saved area_ratio_analysis.png')
for a,r in [(1000,.4),(20000,.4),(1000,.8),(20000,.8)]:
    label,margin,_=evaluate(model,[[np.log(a/(640*480)),r]])
    print(a,r,label[0],float(margin[0]))
