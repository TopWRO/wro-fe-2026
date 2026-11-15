import importlib.util,sys,json,csv
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parent
sp=importlib.util.spec_from_file_location('bidir_work',ROOT/'working_program.py');m=importlib.util.module_from_spec(sp);sys.modules[sp.name]=m;sp.loader.exec_module(m)
rows=[];sheets={};standard={c:[] for c in ['red','green']};counts={};problems=[]
for color in ['red','green']:
 for label,folderlabel in [('tall','tall'),('stand','standard'),('low','short')]:
  folder=Path('C:/Users/xiaoy/Desktop')/f'captures_320x240_{folderlabel}_{color}';files=sorted(folder.glob('*_raw.png'))
  tiles=[];counts[folder.name]=len(files)
  for idx,p in enumerate(files):
   rgb=m.read_rgb(p)
   if rgb.shape!=(240,320,3):raise ValueError((p,rgb.shape))
   prepared,ds=m.extract_color(rgb,color)
   d=max(ds,key=lambda d:d['metrics']['model_area_pixels']) if ds else None
   if len(ds)!=1:problems.append({'file':str(p),'targets':len(ds)})
   shown=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
   if d:
    metrics=d['metrics'];cv2.drawContours(shown,[d['foreground_contour']],-1,(255,0,255),1)
    rows.append({'file':str(p),'folder':folder.name,'color':color,'label':label,'index':idx,'target_count':len(ds),**metrics})
    text='{} A={} b={:.2f} a={:.2f}'.format(idx,metrics['model_area_pixels'],metrics['box_ratio'],metrics['avg_ratio'])
    if label=='stand' and not metrics['partial_view']:
     hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);object_mask=np.zeros(rgb.shape[:2],np.uint8);cv2.drawContours(object_mask,[d['foreground_contour']],-1,255,-1);valid=(prepared[2]!=0)&(object_mask!=0)
     standard[color].append({'file':str(p),'metrics':metrics,'median_hsv':np.median(hsv[valid],axis=0).tolist()})
   else:text=str(idx)+' NO TARGET'
   tile=cv2.resize(shown,(160,120));band=np.full((22,160,3),245,np.uint8);cv2.putText(band,text,(2,15),0,.32,(20,20,20),1)
   tiles.append(np.vstack([band,tile]))
  columns=5;blank=np.full_like(tiles[0],240);grid=[]
  for start in range(0,len(tiles),columns):grid.append(np.hstack((tiles[start:start+columns]+[blank]*columns)[:columns]))
  m.write_png(ROOT/(folder.name+'_contact.png'),np.vstack(grid))
(ROOT/'measurements.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
fields=['file','color','label','index','model_area_pixels','model_area_fraction','box_ratio','avg_ratio','avg_fallback','partial_view','target_count']
with (ROOT/'measurements.csv').open('w',newline='',encoding='utf-8-sig') as f:
 w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
palettes={}
for color,candidates in standard.items():
 median=np.median([d['median_hsv'] for d in candidates],axis=0)
 area_mid=np.quantile([d['metrics']['model_area_pixels'] for d in candidates],.75)
 chosen=min(candidates,key=lambda d:abs(np.log(d['metrics']['model_area_pixels']/area_mid))+.2*abs(d['metrics']['box_ratio']-.5))
 rgb=m.read_rgb(chosen['file']);prepared,ds=m.extract_color(rgb,color);d=max(ds,key=lambda d:d['metrics']['model_area_pixels']);x,y,w,h=d['metrics']['bbox_xywh']
 alpha=np.zeros(rgb.shape[:2],np.uint8);cv2.drawContours(alpha,[d['foreground_contour']],-1,255,-1)
 bgra=np.dstack([cv2.cvtColor(rgb[y:y+h,x:x+w],cv2.COLOR_RGB2BGR),alpha[y:y+h,x:x+w]])
 m.write_png(ROOT/f'standard_{color}_reference.png',bgra)
 palettes[color]={'median_hsv':median.tolist(),'standard_photos_used':len(candidates),'example_source':chosen['file'],'example_bbox':[x,y,w,h]}
(ROOT/'palettes.json').write_text(json.dumps(palettes,indent=2),encoding='utf-8')
report={'total_raw':sum(counts.values()),'measured':len(rows),'counts':counts,'problems':problems,'avg_geometry_fallbacks':sum(r['avg_fallback'] for r in rows),'partial_views':sum(r['partial_view'] for r in rows)}
(ROOT/'extraction_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2));print('PALETTES',palettes)
