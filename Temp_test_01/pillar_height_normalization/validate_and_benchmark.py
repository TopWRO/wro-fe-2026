import importlib.util,json,sys,time,platform
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('height_program',ROOT/'green_texture_standard_height.py');m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
ref=m.standard_reference(m.built_in_reference())
rows=json.loads((ROOT.parent/'pillar_area_model/measurements.json').read_text(encoding='utf-8'))
results=[];examples={};timings=[]
for item in rows:
    row=item['row'];rgb=m.read_rgb(row['file']);original=rgb.copy();tracker=m.Stabilizer(m.Rules())
    out,changed,ds=m.process_frame(rgb,ref,tracker=tracker)
    assert np.array_equal(rgb,original),'Input mutated'
    assert np.array_equal(out[changed==0],rgb[changed==0]),'Unrelated pixels changed'
    assert len(ds)==1
    d=ds[0]['metrics'];label=row['label'];expected={'tall':'TALL-TEXTURE','stand':'NORMAL-KEEP','low':'SHORT-TEXTURE'}[label]
    assert d['decision']==expected,(row,d['decision'])
    if label=='stand':assert np.array_equal(out,rgb),'Standard changed'
    else:
        ax,ay=d['anchor_bottom_xy'];assert d['anchor_preserved'];assert m.process_frame.last_normalized_mask[ay,ax]!=0
        assert not m.process_frame.last_normalized_mask[ay+1:].any()
        assert out[ay,ax,1]>out[ay,ax,0],'Anchor not green'
        if label=='low':assert d['removed_pixels']==0,'Short-pillar original footprint erased'
    for _ in range(3):
        out,changed,ds=m.process_frame(rgb,ref,tracker=tracker);timings.append({'label':label,**m.process_frame.last_timing})
    results.append({'file':row['file'],'label':label,'metrics':ds[0]['metrics']})
    if (label,row['index']) in [('tall',14),('stand',16),('low',14)]:
        outdir=ROOT/'examples'/label
        m.save_result(outdir,rgb,out,changed,ds,ref,m.Settings(),m.Rules(),row['file'])
        pair=np.hstack([cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),cv2.cvtColor(out,cv2.COLOR_RGB2BGR)])
        band=np.full((42,1280,3),240,np.uint8)
        cv2.putText(band,label.upper()+' - ORIGINAL',(14,28),0,.7,(20,20,20),1)
        cv2.putText(band,'STANDARD HEIGHT / GREEN' if label!='stand' else 'STANDARD RED / UNCHANGED',(654,28),0,.7,(20,20,20),1)
        examples[label]=np.vstack([band,pair])
# Slanted silhouettes: lowest image anchor, upward-only extension, both scale directions.
geometry=0
for angle in [-35,-15,0,15,35]:
    mask=np.zeros((480,640),np.uint8);box=cv2.boxPoints(((320,300),(65,180),angle))
    cv2.fillConvexPoly(mask,np.rint(box).astype(np.int32),255)
    for scale in [2/3,2]:
        new,layer,erase,info=m.anchored_height_layer(mask,box,ref,scale)
        ax,ay=info['anchor_bottom_xy'];assert info['anchor_preserved'] and new[ay,ax]
        assert not new[ay+1:].any();geometry+=1
report={'platform':platform.platform(),'processor':platform.processor(),'opencv':cv2.__version__,'image_size':[640,480],
        'frames':len(timings),'photos_checked':len(results),'standard_unchanged':38,'special_anchors_preserved':64,
        'synthetic_slant_checks':geometry,'capture_display_save_excluded':True,'groups':{}}
for label in ['all','tall','stand','low']:
    selected=[r for r in timings if label=='all' or r['label']==label]
    report['groups'][label]={key:{'mean_ms':float(np.mean([r[key] for r in selected])),
        'median_ms':float(np.median([r[key] for r in selected])),
        'p95_ms':float(np.percentile([r[key] for r in selected],95))} for key in ['analysis_ms','height_render_ms','frame_processing_ms']}
(ROOT/'validation.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
(ROOT/'benchmark_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
m.write_png(ROOT/'comparison.png',np.vstack([examples[k] for k in ['tall','stand','low']]))
print(json.dumps(report,indent=2))
