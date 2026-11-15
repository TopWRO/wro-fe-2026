"""Shared extraction/rendering code embedded into the single-file Pi program."""

def color_masks(rgb, settings, color, history=None):
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    c=rgb.astype(np.float32)
    if color=='red':
        strong=cv2.inRange(hsv,(0,settings.s_min,settings.v_min),(12,255,255))|cv2.inRange(hsv,(166,settings.s_min,settings.v_min),(179,255,255))
        weak=cv2.inRange(hsv,(0,settings.weak_s_min,settings.weak_v_min),(18,255,255))|cv2.inRange(hsv,(158,settings.weak_s_min,settings.weak_v_min),(179,255,255))
        plausible=(c[:,:,0]>c[:,:,1]*1.25)&(c[:,:,0]>c[:,:,2]*1.03)
    else:
        strong=cv2.inRange(hsv,(35,settings.s_min,settings.v_min),(90,255,255))
        weak=cv2.inRange(hsv,(30,settings.weak_s_min,settings.weak_v_min),(94,255,255))
        plausible=(c[:,:,1]>c[:,:,0]*1.15)&(c[:,:,1]>c[:,:,2]*1.02)
    strong[~plausible]=0;weak[~plausible]=0
    kernel=np.ones((3,3),np.uint8)
    strong=cv2.morphologyEx(strong,cv2.MORPH_CLOSE,kernel)
    # Keep a 1px-wide dark edge, but reject isolated one-pixel color noise by area.
    weak=cv2.morphologyEx(weak,cv2.MORPH_CLOSE,kernel)
    selected=np.zeros(weak.shape,np.uint8)
    for seed,pad in [(strong,settings.grow_pixels),(history,0)]:
        if seed is None:continue
        n,labels,stats,_=cv2.connectedComponentsWithStats(seed,8)
        for i in range(1,n):
            x,y,w,h,area=stats[i]
            if area<settings.min_area or min(w,h)<settings.min_size:continue
            x0,y0=max(0,x-pad),max(0,y-pad);x1,y1=min(weak.shape[1],x+w+pad),min(weak.shape[0],y+h+pad)
            _,local=cv2.connectedComponents(weak[y0:y1,x0:x1],8)
            hit=np.unique(local[labels[y0:y1,x0:x1]==i]);hit=hit[hit!=0]
            selected[y0:y1,x0:x1]|=np.isin(local,hit).astype(np.uint8)*255
    contours=cv2.findContours(selected,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(selected,contours,-1,255,cv2.FILLED)
    return selected,weak,strong


def measure_target(d,shape):
    m=d['metrics'];m.update(average_edge_geometry(d))
    x,y,w,h=m['bbox_xywh'];local=np.zeros((h,w),np.uint8)
    cv2.drawContours(local,[d['foreground_contour']-np.int32([x,y])],-1,255,-1)
    area=int(cv2.countNonZero(local));fraction=area/(shape[0]*shape[1])
    avg=m.get('average_w_over_h') if m.get('average_quad_valid') else None
    m.update(model_area_pixels=area,model_area_fraction=fraction,log_area=float(np.log(max(fraction,1e-12))),
             box_ratio=float(m['oriented_w_over_h']),avg_ratio=float(avg if avg is not None else m['oriented_w_over_h']),
             avg_fallback=avg is None,partial_view=bool(m['touches_image_border']))
    full=dict(d);full['contour']=d['foreground_contour'];full['box']=cv2.boxPoints(cv2.minAreaRect(full['contour']))
    e=np.roll(full['box'],-1,axis=0)-full['box'];length=np.linalg.norm(e,axis=1)
    horizontal=int(np.argmax(np.abs(e[:,0])/np.maximum(length,1e-6)))
    m['full_box_ratio']=float(length[horizontal]/max(length[(horizontal+1)%4],1e-6))
    avg_full=average_edge_geometry(full)
    m['full_avg_ratio']=float(avg_full['average_w_over_h']) if avg_full.get('average_quad_valid') else m['full_box_ratio']
    m['full_avg_fallback']=not avg_full.get('average_quad_valid',False)
    return m


def extract_color(rgb,color,settings=None,history=None):
    settings=settings or Settings()
    prepared=color_masks(rgb,settings,color,history)
    _,ds=detect_red(rgb,settings,history,prepared)
    for d in ds:
        d['metrics']['source_color']=color
        measure_target(d,rgb.shape)
    return prepared,ds


def straight_mask(region):
    points=cv2.findNonZero(region)
    if points is None:return region.copy()
    hull=cv2.convexHull(points);perimeter=cv2.arcLength(hull,True)
    p=cv2.approxPolyDP(hull,.012*perimeter,True)
    if len(p)<4:p=hull
    flat=points.reshape(-1,2)
    extreme=flat[[np.argmin(flat[:,0]),np.argmax(flat[:,0]),np.argmin(flat[:,1]),np.argmax(flat[:,1])]]
    p=cv2.convexHull(np.vstack([p.reshape(-1,2),extreme]).astype(np.int32))
    result=np.zeros_like(region);cv2.fillConvexPoly(result,p,255)
    # Never move the top/bottom or width beyond the actual observed bbox.
    x,y,w,h=cv2.boundingRect(region)
    result[:y]=0;result[y+h:]=0;result[:,:x]=0;result[:,x+w:]=0
    return result


def infer_exported(model,metrics):
    x=np.array([metrics[k] for k in model['features']]);z=(x-model['mean'])/model['scale']
    sv=np.asarray(model['support_vectors']);dual=np.asarray(model['dual_coef'])
    kernel=np.exp(-model['gamma']*np.sum((sv-z)**2,axis=1))
    starts=np.r_[0,np.cumsum(model['n_support'])];votes=np.zeros(3,int);scores=[];pairs=[];k=0
    for i in range(3):
        for j in range(i+1,3):
            score=float(kernel[starts[i]:starts[i+1]]@dual[j-1,starts[i]:starts[i+1]])
            score+=float(kernel[starts[j]:starts[j+1]]@dual[i,starts[j]:starts[j+1]])+model['intercept'][k];k+=1
            votes[i if score>0 else j]+=1;scores.append(score);pairs.append((i,j))
    winner=int(np.argmax(votes));margin=min(abs(s) for s,p in zip(scores,pairs) if winner in p) if votes[winner]==2 else 0.
    return model['classes'][winner],float(margin)


def paint_other_color(rgb,region,source,target,palettes):
    # Retain THIS object's shading; take target hue/brightness from standard photos.
    x,y,w,h=cv2.boundingRect(region);patch=rgb[y:y+h,x:x+w]
    hsv=cv2.cvtColor(patch,cv2.COLOR_RGB2HSV);selected=region[y:y+h,x:x+w]!=0
    r,g,b=patch[:,:,0].astype(float),patch[:,:,1].astype(float),patch[:,:,2].astype(float)
    valid=selected & (hsv[:,:,1]>=40)&(hsv[:,:,2]>=10)
    valid &= ((r>1.18*g)&(r>b)) if source=='red' else ((g>1.12*r)&(g>b))
    if not valid.any():return None
    _,labels=cv2.distanceTransformWithLabels((~valid).astype(np.uint8),cv2.DIST_L2,5,labelType=cv2.DIST_LABEL_PIXEL)
    lookup=np.zeros((int(labels.max())+1,3),np.uint8);lookup[labels[valid]]=hsv[valid]
    hsvp=lookup[labels];src=np.array(palettes[source]['median_hsv']);dst=np.array(palettes[target]['median_hsv'])
    hsvp[:,:,0]=int(dst[0])
    hsvp[:,:,1]=np.clip(hsvp[:,:,1].astype(float)*dst[1]/max(src[1],1),60,255).astype(np.uint8)
    hsvp[:,:,2]=np.clip(hsvp[:,:,2].astype(float)*dst[2]/max(src[2],1),10,255).astype(np.uint8)
    return x,y,cv2.cvtColor(hsvp,cv2.COLOR_HSV2RGB)


def process_frame(rgb,reference=None,settings=None,rules=None,tracker=None):
    begin=time.perf_counter()
    if rgb.shape!=(240,320,3):raise ValueError('Expected native 320x240 RGB input; got {}'.format(rgb.shape))
    settings=settings or Settings();rules=rules or Rules()
    all_ds=[];masks={};trackers=tracker
    for color in ['red','green']:
        tr=trackers[color] if trackers is not None else None
        hint=tr.hint(rgb.shape) if tr else None
        prepared,ds=extract_color(rgb,color,settings,hint)
        for d in ds:
            m=d['metrics'];model=MODELS[color];label,margin=infer_exported(model,m)
            m.update(model_label=label,model_margin=margin,classification_ratio=m['avg_ratio'],
                     model_outside_training_range=bool(not model['area_range'][0]<=m['model_area_fraction']<=model['area_range'][1]),
                     decision_reason='area_plus_selected_aspect_model')
            m['decision']='UNKNOWN-KEEP' if margin<rules.model_margin else {'tall':'TALL-TEXTURE','stand':'NORMAL-KEEP','low':'SHORT-TEXTURE'}[label]
            m['changed_pixels']=0
        if tr:tr.update(ds,rgb,settings,prepared[1])
        masks[color]=prepared[0];all_ds+=ds
    all_ds.sort(key=lambda d:d['metrics']['bbox_xywh'][0])
    protected=np.zeros(rgb.shape[:2],np.uint8)
    for i,d in enumerate(all_ds,1):
        d['metrics']['id']=i
        if d['metrics']['decision'].endswith('KEEP'):
            cv2.drawContours(protected,[d['foreground_contour']],-1,255,-1)
    out=rgb.copy();changed=np.zeros(rgb.shape[:2],np.uint8)
    for d in all_ds:
        m=d['metrics'];source=m['source_color'];target='green' if source=='red' else 'red'
        m['target_color']=source
        if not m['decision'].endswith('TEXTURE'):continue
        region=d.get('render_mask')
        if region is None:
            region=np.zeros(rgb.shape[:2],np.uint8);cv2.drawContours(region,[d['foreground_contour']],-1,255,-1)
        region=straight_mask(region) if rules.straight_edges else region.copy()
        region[protected!=0]=0
        # Do not recolor another actual opposite-color object, even when touching.
        region[masks[target]!=0]=0
        if not region.any():continue
        result=paint_other_color(rgb,region,source,target,PALETTES if reference is None else reference)
        if result is None:continue
        x,y,paint=result;h,w=paint.shape[:2];selected=region[y:y+h,x:x+w]!=0
        out[y:y+h,x:x+w][selected]=paint[selected];changed|=region
        m['target_color']=target;m['changed_pixels']=int(cv2.countNonZero(region))
        m['render_bbox_xywh']=list(cv2.boundingRect(region))
    process_frame.last_color_masks=masks
    process_frame.last_timing_ms=(time.perf_counter()-begin)*1000
    return out,changed,all_ds
