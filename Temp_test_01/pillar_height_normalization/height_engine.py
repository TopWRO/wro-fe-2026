"""Build-time source embedded into the standalone standard-height program."""
import base64
import time
import cv2
import numpy as np


def standard_reference(green_reference):
    raw=cv2.imdecode(np.frombuffer(base64.b64decode(STANDARD_RED_TEMPLATE_PNG),np.uint8),cv2.IMREAD_UNCHANGED)
    rgb=cv2.cvtColor(raw[:,:,:3],cv2.COLOR_BGR2RGB)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    ref=green_reference['cutout_bgra']
    ghsv=cv2.cvtColor(ref[:,:,:3],cv2.COLOR_BGR2HSV)
    valid=(raw[:,:,3]!=0)&(hsv[:,:,1]>80)
    gvalid=(ref[:,:,3]!=0)&(ghsv[:,:,1]>50)
    source=np.median(hsv[valid],axis=0);target=np.median(ghsv[gvalid],axis=0)
    mapped=hsv.copy()
    mapped[:,:,0]=int(target[0])
    mapped[:,:,1]=np.clip(hsv[:,:,1].astype(np.float32)*target[1]/max(source[1],1),0,255).astype(np.uint8)
    mapped[:,:,2]=np.clip(hsv[:,:,2].astype(np.float32)*target[2]/max(source[2],1),0,255).astype(np.uint8)
    green_rgb=cv2.cvtColor(mapped,cv2.COLOR_HSV2RGB)
    cutout=np.dstack([cv2.cvtColor(green_rgb,cv2.COLOR_RGB2BGR),raw[:,:,3]])
    result=prepare_reference(cutout)
    transform=cv2.getPerspectiveTransform(result['corners'],np.float32([[0,0],[63,0],[63,127],[0,127]]))
    result['canonical_rgb']=cv2.warpPerspective(result['texture'],transform,(64,128),
        flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    return result


def anchored_height_layer(region,box,reference,scale):
    """Scale the observed silhouette along its body axis about a fixed bottom anchor.

    This is a 2-D approximation; it preserves the anchor, transverse width and
    coherent silhouette, rather than treating side-edge pixels as base contacts.
    """
    fh,fw=region.shape
    ys,xs=np.nonzero(region)
    if not len(xs):return None
    bottom_y=int(ys.max());bottom_xs=xs[ys==bottom_y]
    anchor_x=int(bottom_xs[np.argmin(np.abs(bottom_xs-np.median(bottom_xs)))])
    anchor=np.float32([anchor_x,bottom_y])
    corners=ordered_corners(box)
    u=corners[1]-corners[0];u=u/max(float(np.linalg.norm(u)),1e-6)
    v=np.float32([-u[1],u[0]])
    linear=np.eye(2,dtype=np.float32)+(scale-1)*np.outer(v,v)
    affine=np.column_stack([linear,anchor-linear@anchor]).astype(np.float32)
    new_mask=cv2.warpAffine(region,affine,(fw,fh),flags=cv2.INTER_NEAREST)
    if scale>=1:
        # Extension must cover the complete observed short pillar, including base.
        new_mask|=region
    else:
        # Retain the observed base band, avoiding a shifted foot under perspective.
        direction=v if np.dot(v,corners[3]-corners[0])>=0 else -v
        projected_body=xs*direction[0]+ys*direction[1]
        cutoff=projected_body.max()-0.12*(projected_body.max()-projected_body.min())
        base_pixels=projected_body>=cutoff
        new_mask[ys[base_pixels],xs[base_pixels]]=255
    new_mask[bottom_y+1:]=0
    new_mask[bottom_y,anchor_x]=255
    points=cv2.findNonZero(new_mask)
    target=ordered_corners(cv2.boxPoints(cv2.minAreaRect(points)))
    source=np.float32([[0,0],[63,0],[63,127],[0,127]])
    transform=cv2.getPerspectiveTransform(source,target)
    layer=cv2.warpPerspective(reference['canonical_rgb'],transform,(fw,fh),
        flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    erase=cv2.bitwise_and(region,cv2.bitwise_not(new_mask))
    old_bounds=list(cv2.boundingRect(region));new_bounds=list(cv2.boundingRect(new_mask))
    projected=np.column_stack([xs,ys,np.ones(len(xs))])@affine.T
    info={'height_normalized':True,'requested_height_scale':float(scale),
          'anchor_bottom_xy':[anchor_x,bottom_y],
          'anchor_preserved':bool(new_mask[bottom_y,anchor_x]!=0),
          'old_bbox_xywh':old_bounds,'normalized_bbox_xywh':new_bounds,
          'normalized_top_clipped':bool(np.any(projected[:,1]<0)|np.any(projected[:,0]<0)|np.any(projected[:,0]>=fw)),
          'removed_pixels':int(cv2.countNonZero(erase))}
    return new_mask,layer,erase,info


def white_background(rgb,region,mode):
    if mode=='white':return np.uint8([255,255,255])
    ring=cv2.dilate(region,np.ones((21,21),np.uint8))
    ring[region!=0]=0
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    samples=rgb[(ring!=0)&(hsv[:,:,1]<65)&(hsv[:,:,2]>80)]
    return np.median(samples,axis=0).astype(np.uint8) if len(samples)>=20 else np.uint8([235,235,235])


def process_frame(rgb,reference,settings=None,rules=None,tracker=None):
    begin=time.perf_counter()
    settings,rules=settings or Settings(),rules or Rules()
    history=tracker.hint(rgb.shape) if tracker is not None else None
    prepared=seeded_red_mask(rgb,settings,history)
    _,detections=detect_red(rgb,settings,history,prepared)
    detections.sort(key=lambda d:d['metrics']['bbox_xywh'][0])
    for index,d in enumerate(detections,1):
        m=d['metrics'];m['id']=index;m['decision']=classify(d,rules,rgb.shape)
        m['changed_pixels']=0;m['height_normalized']=False
    if tracker is not None:tracker.update(detections,rgb,settings,prepared[1])
    classified=time.perf_counter()
    protected=np.zeros(rgb.shape[:2],np.uint8)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    protected=cv2.inRange(hsv,(40,50,20),(85,255,255))
    for d in detections:
        if d['metrics']['decision'].endswith('KEEP'):
            if 'render_mask' in d:protected|=d['render_mask']
            else:cv2.drawContours(protected,[d['foreground_contour']],-1,255,-1)
    layers=[]
    for d in detections:
        m=d['metrics']
        if not m['decision'].endswith('TEXTURE'):continue
        region=d.get('render_mask')
        if region is None:
            region=np.zeros(rgb.shape[:2],np.uint8)
            cv2.drawContours(region,[d['foreground_contour']],-1,255,-1)
        scale=rules.tall_scale if m['decision']=='TALL-TEXTURE' else rules.low_scale
        result=anchored_height_layer(region,d.get('warp_box',d['box']),reference,scale)
        if result is None:continue
        new_mask,layer,erase,info=result
        new_mask[protected!=0]=0;erase[protected!=0]=0
        m.update(info)
        m['background_fill_rgb']=white_background(rgb,region,rules.background).tolist()
        layers.append((d,new_mask,layer,erase))
    out=rgb.copy();changed=np.zeros(rgb.shape[:2],np.uint8)
    removed=np.zeros(rgb.shape[:2],np.uint8);normalized=np.zeros(rgb.shape[:2],np.uint8)
    # Erase all surplus old pillar pixels before compositing any new pillar.
    for d,new_mask,layer,erase in layers:
        out[erase!=0]=d['metrics']['background_fill_rgb'];removed|=erase
    for d,new_mask,layer,erase in sorted(layers,key=lambda item:item[0]['metrics']['anchor_bottom_xy'][1]):
        out[new_mask!=0]=layer[new_mask!=0]
        normalized|=new_mask;changed|=(new_mask|erase)
        d['metrics']['changed_pixels']=int(cv2.countNonZero(new_mask|erase))
    done=time.perf_counter()
    timing={'analysis_ms':(classified-begin)*1000,'height_render_ms':(done-classified)*1000,
            'frame_processing_ms':(done-begin)*1000}
    for d in detections:d['metrics']['frame_timing_ms']=timing
    process_frame.last_timing=timing
    process_frame.last_removed_mask=removed;process_frame.last_normalized_mask=normalized
    return out,changed,detections
