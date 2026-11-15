import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'pillar_area_model'))
from build_model import evaluate
font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc');plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
rows=json.loads((ROOT/'measurements.json').read_text());models=json.loads((ROOT/'models.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(13.5,5.5),dpi=160,layout='constrained')
colors={'tall':'#2667a2','stand':'#333f4e','low':'#c66c18'};markers={'tall':'^','stand':'o','low':'s'};names={'tall':'高柱','stand':'标准柱','low':'矮柱'}
for ax,color,title in zip(axes,['red','green'],['红色：面积＋核心旋转宽高比','绿色：面积＋完整轮廓旋转宽高比']):
 model=models[color];ratio_key=model['features'][1];sub=[r for r in rows if r['color']==color]
 aa=np.geomspace(70,20000,150);rr=np.linspace(.15,2.9,150);xx,yy=np.meshgrid(aa,rr)
 inputs=np.column_stack([np.log(xx.ravel()/76800),yy.ravel()]);pred,margin,_=evaluate(model,inputs)
 codes=np.array([{'tall':0,'stand':1,'low':2}[p] for p in pred]).reshape(xx.shape)
 from matplotlib.path import Path as PlotPath
 import cv2
 hull=cv2.convexHull(np.float32([[np.log(r['model_area_pixels']),r[ratio_key]] for r in sub])).reshape(-1,2)
 inside=PlotPath(hull).contains_points(np.column_stack([np.log(xx.ravel()),yy.ravel()])).reshape(xx.shape)
 codes=np.ma.masked_where(~inside,codes)
 ax.set_facecolor('#f7f7f7')
 ax.pcolormesh(xx,yy,codes,shading='nearest',cmap=ListedColormap(['#e2eff9','#edf0f2','#fff0dc']),vmin=0,vmax=2,rasterized=True)
 for label in ['tall','stand','low']:
  subset=[r for r in sub if r['label']==label]
  ax.scatter([r['model_area_pixels'] for r in subset],[r[ratio_key] for r in subset],s=30,c=colors[label],marker=markers[label],label=names[label],edgecolors='white',linewidths=.4,zorder=3)
 ax.set_xscale('log');ax.set_xlim(70,20000);ax.set_ylim(.15,2.9);ax.set_xticks([100,300,1000,3000,10000],labels=['100','300','1,000','3,000','10,000'])
 ax.set_xlabel('完整颜色区域像素数 A（320×240）');ax.set_ylabel('宽 ÷ 高');ax.set_title(title,fontweight='bold',pad=12);ax.grid(alpha=.16)
 ax.legend(loc='upper right',framealpha=.95)
fig.suptitle('238张新照片：面积与宽高比联合判断，比固定比例更适合远近变化',fontsize=14,fontweight='bold')
fig.savefig(ROOT/'area_ratio_analysis_320x240.png',facecolor='white')
# Reference-color card; geometric cutouts are independent assets.
import cv2
card=np.full((240,640,3),245,np.uint8)
for color,left in [('red',0),('green',320)]:
 raw=cv2.imdecode(np.fromfile(str(ROOT/f'standard_{color}_reference.png'),np.uint8),cv2.IMREAD_UNCHANGED)
 h,w=raw.shape[:2];scale=min(160/w,160/h);cut=cv2.resize(raw,(round(w*scale),round(h*scale)))
 y=45;x=left+80;hh,ww=cut.shape[:2];patch=card[y:y+hh,x:x+ww];patch[cut[:,:,3]>0]=cut[:,:,:3][cut[:,:,3]>0]
 cv2.putText(card,'STANDARD '+color.upper(),(left+30,25),0,.6,(25,25,25),1)
 cv2.putText(card,'Photo reference / original color',(left+14,225),0,.43,(25,25,25),1)
cv2.imencode('.png',card)[1].tofile(str(ROOT/'standard_references.png'))
print('Analysis plot and reference card written')
