"""Runtime replay, rule/geometry checks, examples and timings."""
import importlib.util,sys,json,platform
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
sp=importlib.util.spec_from_file_location('dual_runtime',ROOT/'green_red_live_area_model_320x240.py');m=importlib.util.module_from_spec(sp);sys.modules[sp.name]=m;sp.loader.exec_module(m)
rows=json.loads((ROOT/'measurements.json').read_text());results=[];times=[];examples={};standard_exact=0;errors=[]
for r in rows:
 rgb=m.read_rgb(r['file']);original=rgb.copy();tracker={c:m.Stabilizer(m.Rules()) for c in ['red','green']}
 out,mask,ds=m.process_frame(rgb,tracker=tracker)
 assert out.shape==(240,320,3) and mask.shape==(240,320)
 assert np.array_equal(original,rgb) and np.array_equal(out[mask==0],rgb[mask==0])
 matched=[d for d in ds if d['metrics']['source_color']==r['color']]
 assert matched
 d=max(matched,key=lambda d:d['metrics']['model_area_pixels']);mm=d['metrics']
 if mm['model_label']!=r['label']:errors.append({'file':r['file'],'truth':r['label'],'prediction':mm['model_label']})
 primary=np.zeros(mask.shape,np.uint8);cv2.drawContours(primary,[d['foreground_contour']],-1,255,-1)
 if r['label']=='stand':
  assert np.array_equal(out[primary!=0],rgb[primary!=0]),('Standard target modified',r['file'])
  standard_exact+=1
 elif mm['decision'].endswith('TEXTURE'):
  assert mm['target_color']!=r['color']
  x,y,w,h=mm['render_bbox_xywh'];ox,oy,ow,oh=mm['bbox_xywh']
  assert (x,y,w,h)==(ox,oy,ow,oh),('Height/width bbox moved',r['file'],(x,y,w,h),(ox,oy,ow,oh))
 for _ in range(2):
  out,mask,ds=m.process_frame(rgb,tracker=tracker);times.append(m.process_frame.last_timing_ms)
 results.append({'file':r['file'],'truth':r['label'],'color':r['color'],'primary':mm,'targets':len(ds)})
 # Moderate distances, all six classes. These are replay illustrations, not validation.
 if (r['color'],r['label'],r['index']) in [('red','tall',7),('red','stand',18),('red','low',14),('green','tall',7),('green','stand',18),('green','low',18)]:
  key=r['color']+'_'+r['label'];prefix=m.save_result(ROOT/'examples'/key,rgb,out,mask,ds,m.PALETTES,m.Settings(),m.Rules(),r['file'])
  band=np.full((24,640,3),240,np.uint8)
  cv2.putText(band,key.upper()+' ORIGINAL',(5,17),0,.42,(20,20,20),1)
  cv2.putText(band,'UNCHANGED' if r['label']=='stand' else 'COLOR SWAPPED / SAME HEIGHT',(325,17),0,.40,(20,20,20),1)
  examples[key]=np.vstack([band,np.hstack([cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),cv2.cvtColor(out,cv2.COLOR_RGB2BGR)])])
# Simultaneous bidirectional changes must read ORIGINAL pixels, not each other's output.
original_infer=m.infer_exported
synthetic=np.full((240,320,3),220,np.uint8);synthetic[40:180,30:70]=[180,10,15];synthetic[40:180,180:220]=[12,100,30]
for label in ['tall','low','stand']:
 m.infer_exported=lambda model,metrics,label=label:(label,1.)
 out,mask,ds=m.process_frame(synthetic)
 if label=='stand':assert np.array_equal(out,synthetic)
 else:
  assert out[100,50,1]>out[100,50,0] and out[100,200,0]>out[100,200,1]
  assert np.array_equal(out[mask==0],synthetic[mask==0])
m.infer_exported=original_infer
# Border clipping and a convex hull with notches retain observed extent.
for x in [0,40,290]:
 region=np.zeros((240,320),np.uint8);cv2.rectangle(region,(x,0),(min(x+29,319),180),255,-1);region[80:90,x:x+7]=0
 straight=m.straight_mask(region);assert cv2.boundingRect(straight)==cv2.boundingRect(region)
# A standard pillar next to a special pillar must remain protected.
def mixed(model,metrics):return ('stand' if metrics['source_color']=='green' else 'tall',1.)
m.infer_exported=mixed;out,mask,ds=m.process_frame(synthetic)
assert np.array_equal(out[40:180,180:220],synthetic[40:180,180:220]);assert out[100,50,1]>out[100,50,0]
m.infer_exported=original_infer
# Camera settings and UI control flow checked without a real camera/window.
for fn in ['namedWindow','imshow','destroyWindow','destroyAllWindows']:setattr(m.cv2,fn,lambda *a,**kw:None)
m.cv2.getWindowProperty=lambda *a:1
keys=iter([ord('d'),ord('c'),ord('s'),ord('q')]);m.cv2.waitKey=lambda *a:next(keys)
args=m.build_parser().parse_args([]);preview=m.DualPreview(args,m.Settings(),m.Rules());saved=[]
original_save=m.save_result;m.save_result=lambda *a,**kw:saved.append(True)
for _ in range(3):preview.run(synthetic)
try:preview.run(synthetic)
except KeyboardInterrupt:pass
else:raise AssertionError('Q failed')
assert saved;m.save_result=original_save
report={'raw_images':len(rows),'standard_targets_unchanged':standard_exact,'primary_replay_errors':errors,
 'synthetic_both_directions_and_standards':True,'same_bbox_and_height':True,'ui_keys_mocked':True,
 'platform':platform.platform(),'opencv':cv2.__version__,'timed_frames':len(times),
 'mean_ms':float(np.mean(times)),'median_ms':float(np.median(times)),'p95_ms':float(np.percentile(times,95)),
 'capture_display_save_excluded':True}
(ROOT/'runtime_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(ROOT/'runtime_replay.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
order=['red_tall','red_stand','red_low','green_tall','green_stand','green_low']
m.write_png(ROOT/'comparison_six_cases.png',np.vstack([examples[k] for k in order]))
print(json.dumps(report,indent=2))
