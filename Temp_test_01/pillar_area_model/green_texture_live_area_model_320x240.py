#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Area/aspect classifier learned from labeled tall/stand/low photos. Thonny single-file version.
320x240 photo-collection variant; original pillar heights; no height normalization.
Run in the SAME Raspberry Pi Python environment as Recording_N02.py.
S: save; R: sample green pillar; D: debug/mask; C: clear memory; Q/Esc: quit.
"""
import argparse
import base64
from itertools import combinations
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time
import cv2
import numpy as np

# Native camera acquisition size for the new photo collection.
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

REFERENCE_PNG_BASE64 = (
    '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'
)

AREA_MODEL = json.loads('{\n  "features": [\n    "log_area_fraction",\n    "rotated_box_width_over_height"\n  ],\n  "classes": [\n    "low",\n    "stand",\n    "tall"\n  ],\n  "mean": [\n    -4.359456660352639,\n    0.586020598065794\n  ],\n  "scale": [\n    1.1981183170764895,\n    0.23803434388625336\n  ],\n  "support_vectors": [\n    [\n      -1.756167978156177,\n      0.7389043350370674\n    ],\n    [\n      -0.9318826857015418,\n      0.35970386897096823\n    ],\n    [\n      -0.7674682293448014,\n      0.4062123537463623\n    ],\n    [\n      -0.13652212742428893,\n      0.8601012467259924\n    ],\n    [\n      1.0041733254866876,\n      2.4086134279481857\n    ],\n    [\n      1.3175261454339218,\n      1.7660591787348252\n    ],\n    [\n      0.3204493686668355,\n      1.1455273204390457\n    ],\n    [\n      0.022298380911437387,\n      0.9559069482768487\n    ],\n    [\n      -1.24852086067829,\n      1.1369085642365973\n    ],\n    [\n      -1.189041743294142,\n      -0.7703815518746261\n    ],\n    [\n      -0.5463587758932321,\n      -0.7461443852477783\n    ],\n    [\n      -0.3638023504673666,\n      -0.7100747728754965\n    ],\n    [\n      -0.15014718027253776,\n      -0.6726895627489886\n    ],\n    [\n      0.2769213745521276,\n      -0.4388319590437182\n    ],\n    [\n      0.5702749762476567,\n      -0.24702191769005805\n    ],\n    [\n      1.0367711453269326,\n      0.37217229081611536\n    ],\n    [\n      1.6320371997921912,\n      1.6837407003346982\n    ],\n    [\n      1.7577585328613634,\n      2.2207682103017783\n    ],\n    [\n      1.467204439858934,\n      1.2095250996065359\n    ],\n    [\n      1.346093534261856,\n      0.9431987339071327\n    ],\n    [\n      1.5954305576613226,\n      1.4585791644692712\n    ],\n    [\n      1.4246261877311612,\n      0.934338357914412\n    ],\n    [\n      0.8880515422352293,\n      0.030000217832075755\n    ],\n    [\n      -0.8311277382426118,\n      -0.5714335318130925\n    ],\n    [\n      -0.8802907915589329,\n      -1.1842748416570477\n    ],\n    [\n      0.48908165641574575,\n      -1.0609932508211404\n    ],\n    [\n      0.7594764408787875,\n      -0.9497398800562732\n    ],\n    [\n      1.1312373058741942,\n      -0.6692010854354171\n    ],\n    [\n      1.5304386940671728,\n      -0.16613777201341856\n    ],\n    [\n      1.6979299943808241,\n      0.11966012241372216\n    ],\n    [\n      2.223990793607819,\n      1.0381020076457592\n    ],\n    [\n      1.5453175640886736,\n      -0.27605784546137657\n    ],\n    [\n      1.1234751856561438,\n      -0.6660353610357755\n    ],\n    [\n      0.7700169233228153,\n      -0.8942447484193528\n    ],\n    [\n      -0.37838741295593975,\n      -0.9946291771666154\n    ],\n    [\n      -0.4687070275531714,\n      -1.0043176248961057\n    ]\n  ],\n  "dual_coef": [\n    [\n      0.7632575894679541,\n      1.1599905342992032,\n      0.0,\n      0.7108486462081981,\n      0.0,\n      10.0,\n      0.0,\n      0.0,\n      0.02627934518390547,\n      -0.1591692701646912,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.13277326437794082,\n      -0.5201100542260493,\n      -0.20152172562614082,\n      -7.735398916794153,\n      -1.3474601007789402,\n      -0.9829844195770796,\n      -0.4657170501592095,\n      -0.0,\n      -0.0,\n      -0.0,\n      -1.1152413134550574,\n      -0.861424679702496,\n      -0.5063305274123161,\n      -0.17132263981469714,\n      -0.06940082917246143,\n      -0.15607334146830143,\n      -0.1029606757286259,\n      -1.1418896845339839,\n      -0.4399247539996257,\n      -0.0,\n      -0.0,\n      -0.11667265710901638,\n      -0.0\n    ],\n    [\n      0.6380005684286096,\n      0.3392600352776815,\n      0.26412130597906075,\n      0.0,\n      0.4299691474708549,\n      0.9544198774251231,\n      0.3382179289495795,\n      0.38962083645021006,\n      0.21239008896040415,\n      0.26865929729340776,\n      10.0,\n      1.238111819376442,\n      2.1892224631944828,\n      0.0,\n      0.0,\n      0.0,\n      0.0,\n      0.8994332040952729,\n      0.0,\n      0.0,\n      0.2724779229042939,\n      1.9793287200975573,\n      1.7084085626896748,\n      0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -0.0,\n      -1.7693408662888686,\n      -1.8726176548009246,\n      -0.0,\n      -1.227067948346663,\n      -0.4310137322044354,\n      -3.2556017880102375,\n      -10.0\n    ]\n  ],\n  "intercept": [\n    -0.24208265253605704,\n    -0.024452104180349113,\n    0.0004732686129508967\n  ],\n  "n_support": [\n    9,\n    15,\n    12\n  ],\n  "gamma": 1.0,\n  "training_samples": 102,\n  "training_area_fraction_range": [\n    0.0015592447916666665,\n    0.210400390625\n  ],\n  "training_box_ratio_range": [\n    0.28125,\n    1.15935331506306\n  ],\n  "default_margin": 0.15,\n  "dataset_class_counts": {\n    "low": 31,\n    "stand": 38,\n    "tall": 33\n  }\n}')


@dataclass
class Settings:
    # Same red HSV defaults as Recording_N02.py, including the hue wraparound.
    hue_width: int = 10
    s_min: int = 100
    v_min: int = 70
    min_area: int = 150
    min_size: int = 12
    kernel: int = 3
    weak_s_min: int = 55
    weak_v_min: int = 10
    grow_pixels: int = 4


def seeded_red_mask(rgb, settings, history=None):
    """Strong red seeds select weak-red connected components; fill their holes.

    White/black alone are not positive evidence. Original green is excluded.
    History can seed a dark component, but cannot paint absent/gray background.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    strong = cv2.bitwise_or(
        cv2.inRange(hsv, (0,settings.s_min,settings.v_min), (settings.hue_width,255,255)),
        cv2.inRange(hsv, (180-settings.hue_width,settings.s_min,settings.v_min),(179,255,255)))
    weak = cv2.bitwise_or(
        cv2.inRange(hsv,(0,settings.weak_s_min,settings.weak_v_min),(15,255,255)),
        cv2.inRange(hsv,(165,settings.weak_s_min,settings.weak_v_min),(179,255,255)))
    c = rgb.astype(np.float32)
    weak[(c[:,:,0] <= c[:,:,1]*1.30) | (c[:,:,0] < c[:,:,2]*1.05)] = 0
    weak = cv2.morphologyEx(weak,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    strong = cv2.morphologyEx(strong,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    strong = cv2.morphologyEx(strong,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    selected = np.zeros(weak.shape,np.uint8)
    def grow_local(seed_image,pad,minimum):
        count,labels,stats,_=cv2.connectedComponentsWithStats(seed_image,8)
        for i in range(1,count):
            x,y,w,h,area=stats[i]
            if area<minimum or min(w,h)<settings.min_size:continue
            x0,y0=max(0,x-pad),max(0,y-pad)
            x1,y1=min(weak.shape[1],x+w+pad),min(weak.shape[0],y+h+pad)
            _,local_labels=cv2.connectedComponents(weak[y0:y1,x0:x1],8)
            hit=np.unique(local_labels[labels[y0:y1,x0:x1]==i])
            hit=hit[hit!=0]
            # Bounded growth cannot run along a dark track stripe/wood surface.
            local=np.isin(local_labels,hit).astype(np.uint8)*255
            selected[y0:y1,x0:x1] |= local
    grow_local(strong,settings.grow_pixels,settings.min_area)
    if history is not None:
        # History only retains inside its predicted footprint; it cannot expand
        # by another margin every frame and creep indefinitely into background.
        grow_local(history,0,settings.min_area)
    # Only close small gaps. Fill holes within each selected component, not hulls
    # spanning separate targets and not a whole bounding rectangle.
    selected = cv2.morphologyEx(selected,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours = cv2.findContours(selected,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    filled = np.zeros(selected.shape,np.uint8)
    cv2.drawContours(filled,contours,-1,255,cv2.FILLED)
    return filled, weak, strong

def detect_red(rgb, settings, history=None, prepared=None):
    """Return cleaned mask and up to twenty largest independent red regions.

    Do not swap RGB channels, resize the frame, or merge distant components.
    minAreaRect compensates for modest image-plane roll, NOT 3-D perspective.
    """
    mask, _, strong = prepared if prepared is not None else seeded_red_mask(rgb,settings,history)
    cores = cv2.findContours(strong,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    core_info=[]
    for core in cores:
        moments=cv2.moments(core)
        if moments['m00']>=settings.min_area:
            center=(moments['m10']/moments['m00'],moments['m01']/moments['m00'])
            core_info.append((cv2.contourArea(core),core,center))

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
    frame_h, frame_w = mask.shape
    detections = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        foreground = contour
        area = float(cv2.contourArea(contour))
        x, y, w, h = cv2.boundingRect(contour)
        if area < settings.min_area or min(w, h) < settings.min_size:
            continue
        foreground_bbox=[int(x),int(y),int(w),int(h)]
        matches=[entry for entry in core_info if cv2.pointPolygonTest(foreground,entry[2],False)>=0]
        if matches:
            _,contour,_=max(matches,key=lambda entry:entry[0])
            area=float(cv2.contourArea(contour))
            x,y,w,h=cv2.boundingRect(contour)
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        edges = np.roll(box, -1, axis=0) - box
        lengths = np.linalg.norm(edges, axis=1)
        if min(lengths) < 1:
            continue
        # Preserve screen-horizontal width and screen-vertical height. Sorting
        # edges into long/short would lose W/H>1 for the short, wide model.
        horizontal = int(np.argmax(np.abs(edges[:, 0]) / lengths))
        width = float(lengths[horizontal])
        height = float(lengths[(horizontal + 1) % 4])
        polygon = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        vertices = len(polygon)
        outline = "QUAD" if vertices == 4 and cv2.isContourConvex(polygon) else (
            "POLYGON-{}".format(vertices) if 3 <= vertices <= 6 else "IRREGULAR/ROUND")
        fx,fy,fw,fh=foreground_bbox
        clipped = fx<=0 or fy<=0 or fx+fw>=frame_w or fy+fh>=frame_h
        metrics = {
            "id": len(detections) + 1,
            "bbox_xywh": foreground_bbox,
            "shape_bbox_xywh": [int(x),int(y),int(w),int(h)],
            "shape_from_strong_core": bool(matches),
            "axis_width_px": int(w), "axis_height_px": int(h),
            "axis_w_over_h": float(w / h),
            "oriented_width_px": width, "oriented_height_px": height,
            "oriented_w_over_h": width / height,
            "oriented_h_over_w": height / width,
            "contour_area_px": area,
            "rectangle_fill": float(area / (width * height)),
            "polygon_vertices": vertices, "outline": outline,
            "touches_image_border": bool(clipped),
        }
        detections.append({"metrics": metrics, "contour": contour,
                           "foreground_contour": foreground,"box":box,"polygon":polygon})
        if len(detections) == 20:
            break
    return mask, detections


@dataclass
class Rules:
    model_margin: float = 0.15  # SVM decision margin; not a probability.
    edge_pixels: int = 2
    edge_s_min: int = 40
    edge_v_min: int = 15
    switch_frames: int = 5


def ordered_corners(box):
    top, bottom = np.array_split(box[np.argsort(box[:, 1])], 2)
    top, bottom = top[np.argsort(top[:, 0])], bottom[np.argsort(bottom[:, 0])]
    return np.float32([top[0], top[1], bottom[1], bottom[0]])






def average_edge_geometry(detection):
    """Approximate the actual silhouette with a maximum-area inscribed quad.

    Do NOT measure minAreaRect edges: opposite rectangle edges are already equal.
    A multi-face 5/6-corner silhouette has no unique true 4-corner interpretation;
    this is an explicit approximation, with fit quality reported for inspection.
    """
    hull = cv2.convexHull(detection['contour'])
    perimeter = cv2.arcLength(hull, True)
    points = cv2.approxPolyDP(hull, .015 * perimeter, True).reshape(-1,2)
    epsilon = .015
    while len(points) > 12 and epsilon < .10:
        epsilon += .005
        points = cv2.approxPolyDP(hull, epsilon*perimeter, True).reshape(-1,2)
    if len(points) < 4 or len(points) > 12:
        return {'average_quad_valid': False, 'average_w_over_h': None,
                'average_quad_points': [], 'quad_fit_fraction': 0.0}
    quad = max((points[list(ids)].astype(np.float32) for ids in combinations(range(len(points)),4)),
               key=cv2.contourArea)
    # Order top/bottom in the upright box coordinate system, compensating roll.
    matrix = cv2.getPerspectiveTransform(ordered_corners(detection['box']),
                                         np.float32([[0,0],[1,0],[1,1],[0,1]]))
    local = cv2.perspectiveTransform(quad.reshape(-1,1,2), matrix).reshape(-1,2)
    indices = np.argsort(local[:,1])
    upper, lower = indices[:2], indices[2:]
    upper = upper[np.argsort(local[upper,0])]
    lower = lower[np.argsort(local[lower,0])]
    quad = quad[[upper[0],upper[1],lower[1],lower[0]]]
    tl,tr,br,bl = quad
    top, bottom = float(np.linalg.norm(tr-tl)), float(np.linalg.norm(br-bl))
    left, right = float(np.linalg.norm(bl-tl)), float(np.linalg.norm(br-tr))
    width, height = (top+bottom)/2.0, (left+right)/2.0
    fit = float(cv2.contourArea(quad) / max(cv2.contourArea(hull),1.0))
    valid = bool(cv2.isContourConvex(quad) and min(top,bottom,left,right)>3 and fit >= .75)
    return {'average_quad_valid': valid, 'average_quad_points': quad.astype(int).tolist(),
            'edge_top_width_px': top, 'edge_bottom_width_px': bottom,
            'edge_left_height_px': left, 'edge_right_height_px': right,
            'average_width_px': width, 'average_height_px': height,
            'average_w_over_h': width/height if height else None,
            'average_h_over_w': height/width if width else None,
            'quad_fit_fraction': fit}


def infer_area_model(area_fraction, box_ratio):
    """Exact NumPy-only evaluation of the exported 36-support-vector model."""
    model=AREA_MODEL
    z=(np.array([np.log(max(area_fraction,1e-12)),box_ratio])-model['mean'])/model['scale']
    sv=np.asarray(model['support_vectors'])
    kernel=np.exp(-model['gamma']*np.sum((sv-z)**2,axis=1))
    dual=np.asarray(model['dual_coef']);starts=np.r_[0,np.cumsum(model['n_support'])]
    votes=np.zeros(3,int);pair_scores=[];pairs=[];k=0
    for i in range(3):
        for j in range(i+1,3):
            score=float(kernel[starts[i]:starts[i+1]]@dual[j-1,starts[i]:starts[i+1]])
            score+=float(kernel[starts[j]:starts[j+1]]@dual[i,starts[j]:starts[j+1]])
            score+=model['intercept'][k];k+=1
            votes[i if score>0 else j]+=1
            pair_scores.append(score);pairs.append((i,j))
    winner=int(np.argmax(votes))
    margin=min(abs(s) for s,pair in zip(pair_scores,pairs) if winner in pair) if votes[winner]==2 else 0.
    return model['classes'][winner],float(margin)


def classify(detection, rules, frame_shape):
    m = detection['metrics']
    m.update(average_edge_geometry(detection))
    # Identical feature extraction to the training script: filled foreground
    # pixels, normalized by frame size, plus the strong-core rotated-box ratio.
    x,y,w,h=m['bbox_xywh']
    local=np.zeros((h,w),np.uint8)
    contour=detection['foreground_contour']-np.int32([x,y])
    cv2.drawContours(local,[contour],-1,255,cv2.FILLED)
    area=int(cv2.countNonZero(local));fraction=area/(frame_shape[0]*frame_shape[1])
    ratio=float(m['oriented_w_over_h'])
    label,margin=infer_area_model(fraction,ratio)
    m['model_area_pixels']=area;m['model_area_fraction']=fraction
    m['model_label']=label;m['model_margin']=margin
    ar=AREA_MODEL['training_area_fraction_range'];rr=AREA_MODEL['training_box_ratio_range']
    m['model_outside_training_range']=bool(not ar[0]<=fraction<=ar[1] or not rr[0]<=ratio<=rr[1])
    m['classification_ratio']=ratio
    m['decision_reason']='learned_area_and_box_ratio'
    m['partial_view'] = m['touches_image_border']
    if margin < rules.model_margin:
        m['decision_reason']='model_margin_low_hold_previous'
        return 'UNKNOWN-KEEP'
    return {'tall':'TALL-TEXTURE','stand':'NORMAL-KEEP','low':'SHORT-TEXTURE'}[label]


class Stabilizer:
    """Greedy spatial association, motion-aligned masks and confirmed model class changes.

    Tracks never render unmatched historical masks, avoiding green ghost trails.
    It is not an occlusion-aware identity tracker; crossing/merged objects can
    be ambiguous and are part of the real-video validation still needed.
    """
    def __init__(self, rules):
        self.rules = rules
        self.reset()

    def reset(self):
        self.tracks, self.next_id = {}, 1
        self.last_mask = None

    def hint(self, shape):
        hint = np.zeros(shape[:2],np.uint8)
        for t in self.tracks.values():
            if t['missed'] <= 3:
                dx,dy = t['velocity']
                matrix = np.float32([[1,0,dx],[0,1,dy]])
                predicted = cv2.warpAffine(t['active'].astype(np.uint8)*255,
                                          matrix,(shape[1],shape[0]))
                hint = cv2.bitwise_or(hint,predicted)
        return hint

    @staticmethod
    def association(a,b):
        ax,ay,aw,ah = a; bx,by,bw,bh = b
        inter = max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
        iou = inter/max(aw*ah+bw*bh-inter,1)
        distance = np.hypot(ax+aw/2-bx-bw/2,ay+ah/2-by-bh/2)
        scale = max(np.hypot(aw,ah),np.hypot(bw,bh),1)
        area_change = aw*ah/max(bw*bh,1)
        if not .25 <= area_change <= 4 or (iou < .08 and distance > .35*scale):
            return None
        return float(iou - .25*distance/scale)

    def update(self,detections,rgb,settings,weak=None):
        fh,fw = rgb.shape[:2]
        if weak is None:_,weak,_ = seeded_red_mask(rgb,settings)
        pairs=[]
        for i,d in enumerate(detections):
            for tid,t in self.tracks.items():
                predicted = list(t['bbox'])
                predicted[0] += t['velocity'][0]*(t['missed']+1)
                predicted[1] += t['velocity'][1]*(t['missed']+1)
                score = self.association(d['metrics']['bbox_xywh'],predicted)
                if score is not None:pairs.append((score,i,tid))
        assigned,used = {},set()
        for _,i,tid in sorted(pairs,reverse=True):
            if i not in assigned and tid not in used:assigned[i]=tid;used.add(tid)
        for t in self.tracks.values():t['missed']+=1
        self.last_mask = np.zeros((fh,fw),np.uint8)
        for i,d in enumerate(detections):
            m=d['metrics']; raw=m['decision']; ratio=m['classification_ratio']
            current=np.zeros((fh,fw),np.uint8)
            cv2.drawContours(current,[d['foreground_contour']],-1,255,cv2.FILLED)
            bbox=m['bbox_xywh']; center=np.array([bbox[0]+bbox[2]/2,bbox[1]+bbox[3]/2])
            if i not in assigned:
                tid=self.next_id;self.next_id+=1
                t={'bbox':bbox,'center':center,'velocity':np.zeros(2),'missed':0,
                   'prob':current.astype(np.float32)/255,'active':current!=0,'box':ordered_corners(d['box']),
                   'ratios':deque(maxlen=7),'state':raw,'pending':None,'votes':0}
                self.tracks[tid]=t
                previous_active=current!=0
            else:
                tid=assigned[i];t=self.tracks[tid]
                delta=center-t['center']
                # A one-pixel contour protrusion must not move the whole history
                # mask. Small real motion is absorbed by the current-mask update.
                delta[np.abs(delta)<2.0]=0
                matrix=np.float32([[1,0,delta[0]],[0,1,delta[1]]])
                previous=cv2.warpAffine(t['prob'],matrix,(fw,fh))
                previous_active=cv2.warpAffine(t['active'].astype(np.uint8),matrix,(fw,fh),
                                               flags=cv2.INTER_NEAREST)!=0
                t['prob']=.65*previous+.35*(current.astype(np.float32)/255)
                t['velocity']=.5*t['velocity']+.5*delta
                shifted=t['box']+delta
                t['box']=.65*shifted+.35*ordered_corners(d['box'])
                t['center'],t['bbox'],t['missed']=center,bbox,0
            if ratio is not None and raw!='UNKNOWN-KEEP' and not m['partial_view']:
                t['ratios'].append(float(ratio))
            measured=float(np.median(t['ratios'])) if t['ratios'] else ratio
            proposal=t['state']
            if raw!='UNKNOWN-KEEP' and not (m['partial_view'] and t['state']!='UNKNOWN-KEEP'):
                proposal=raw  # Model margin gates evidence; no fixed ratio threshold.
            if proposal!=t['state']:
                t['votes']=t['votes']+1 if t['pending']==proposal else 1
                t['pending']=proposal
                if t['votes']>=self.rules.switch_frames:
                    t['state']=proposal;t['pending']=None;t['votes']=0
            else:t['pending']=None;t['votes']=0
            # Keep moving mask memory only where the CURRENT image supports it.
            # Interior holes are already filled. Strong core comes in immediately.
            near=cv2.dilate(current,np.ones((2*self.rules.edge_pixels+1,2*self.rules.edge_pixels+1),np.uint8))
            allowed=(current!=0)|((near!=0)&(weak!=0))
            stable=((t['prob']>=.55)|(previous_active&(t['prob']>=.25))) & allowed
            t['prob'][~allowed]=0
            t['active']=stable
            d['render_mask']=stable.astype(np.uint8)*255
            d['warp_box']=t['box'].astype(np.float32)
            self.last_mask=cv2.bitwise_or(self.last_mask,d['render_mask'])
            m['raw_decision']=raw;m['decision']=t['state'];m['track_id']=tid
            m['filtered_ratio']=measured;m['pending_votes']=t['votes']
            m['decision_held']=bool(t['state']!=raw)
        self.tracks={tid:t for tid,t in self.tracks.items() if t['missed']<=6}


def prepare_reference(bgra):
    """Keep real photographed shading; extend exterior with nearest green pixel."""
    alpha = bgra[:, :, 3]
    contours = cv2.findContours(alpha, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours or cv2.countNonZero(alpha) < 100:
        raise ValueError('Green reference is too small')
    contour = max(contours, key=cv2.contourArea)
    if min(cv2.minAreaRect(contour)[1]) < 8:
        raise ValueError('Green reference is too narrow')
    rgb = cv2.cvtColor(bgra[:, :, :3], cv2.COLOR_BGR2RGB)
    _, labels = cv2.distanceTransformWithLabels(
        np.where(alpha != 0, 0, 255).astype(np.uint8), cv2.DIST_L2, 5,
        labelType=cv2.DIST_LABEL_PIXEL)
    texture = rgb[alpha != 0][labels - 1]
    corners = ordered_corners(cv2.boxPoints(cv2.minAreaRect(contour)))
    return {'texture': texture, 'corners': corners, 'cutout_bgra': bgra.copy()}


def built_in_reference():
    """The actual third green pillar in the user's approved photograph."""
    data = np.frombuffer(base64.b64decode(REFERENCE_PNG_BASE64), np.uint8)
    return prepare_reference(cv2.imdecode(data, cv2.IMREAD_UNCHANGED))


def sample_reference(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (40, 60, 30), (85, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    fh, fw = mask.shape
    candidates = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if cv2.contourArea(c) >= 150 and min(w, h) >= 12 and x > 0 and y > 0 and x+w < fw and y+h < fh:
            candidates.append(c)
    if not candidates:
        raise ValueError('No complete real green pillar found; existing reference retained')
    c = max(candidates, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    inside = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(inside, [c], -1, 255, cv2.FILLED)
    alpha = cv2.bitwise_and(inside, mask)[y:y+h, x:x+w]
    bgra = np.dstack([cv2.cvtColor(rgb[y:y+h, x:x+w], cv2.COLOR_RGB2BGR), alpha])
    return prepare_reference(bgra)


def process_frame(rgb, reference, settings=None, rules=None, tracker=None):
    """RGB in, clean RGB out. No input mutation or feedback from converted pixels.

    Geometry uses a strong red core. Bounded weak-color growth and filled holes
    provide foreground coverage. Optional tracking stabilizes mask and class.
    """
    settings, rules = settings or Settings(), rules or Rules()
    history = tracker.hint(rgb.shape) if tracker is not None else None
    prepared=seeded_red_mask(rgb,settings,history)
    _, detections = detect_red(rgb,settings,history,prepared)
    detections.sort(key=lambda d: d['metrics']['bbox_xywh'][0])
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    edge_hue = max(settings.hue_width, 20)
    edge_red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, rules.edge_s_min, rules.edge_v_min), (edge_hue, 255, 255)),
        cv2.inRange(hsv, (160, rules.edge_s_min, rules.edge_v_min), (179, 255, 255)))
    # Slightly purple dark-red fringe is allowed; green/gray background isn't.
    channels = rgb.astype(np.float32)
    plausible_red = (channels[:, :, 0] > channels[:, :, 1]*1.10) & (
        channels[:, :, 0] >= channels[:, :, 2]*0.85)
    edge_red[~plausible_red] = 0
    for index, d in enumerate(detections, 1):
        m = d['metrics']
        m['id'] = index
        decision = classify(d, rules, rgb.shape)
        m['decision'] = decision
        m['changed_pixels'] = 0
    if tracker is not None:
        tracker.update(detections,rgb,settings,prepared[1])
    protected = np.zeros(hsv.shape[:2],np.uint8)
    for d in detections:
        m = d['metrics']
        if m['decision'].endswith('KEEP'):
            if 'render_mask' in d:protected=cv2.bitwise_or(protected,d['render_mask'])
            else:cv2.drawContours(protected,[d['foreground_contour']],-1,255,cv2.FILLED)
    out = rgb.copy()
    changed = np.zeros(hsv.shape[:2], np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                      (2*rules.edge_pixels+1, 2*rules.edge_pixels+1))
    for d in detections:
        if not d['metrics']['decision'].endswith('TEXTURE'):
            continue
        region = np.zeros(hsv.shape[:2], np.uint8)
        if 'render_mask' in d:region=d['render_mask'].copy()
        else:cv2.drawContours(region,[d['foreground_contour']],-1,255,cv2.FILLED)
        # Fill the whole detected silhouette, including very dark internal gaps.
        # Outside it, expand only into nearby plausible dark-red pixels.
        if rules.edge_pixels and tracker is None:
            ring_red = cv2.bitwise_and(cv2.dilate(region, kernel), edge_red)
            region = cv2.bitwise_or(region, ring_red)
        region[protected != 0] = 0
        x, y, w, h = cv2.boundingRect(region)
        if not w or not h:
            continue
        destination = d.get('warp_box',ordered_corners(d['box'])) - np.float32([x,y])
        transform = cv2.getPerspectiveTransform(reference['corners'], destination)
        fitted = cv2.warpPerspective(reference['texture'], transform, (w, h),
                                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        selected = region[y:y+h, x:x+w] != 0
        out[y:y+h, x:x+w][selected] = fitted[selected]
        changed = cv2.bitwise_or(changed, region)
        d['metrics']['changed_pixels'] = int(cv2.countNonZero(region))
    return out, changed, detections


def debug_view(rgb, detections):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    fh,fw = rgb.shape[:2]
    view = np.full((fh+36+24*len(detections), max(fw,640),3),28,np.uint8)
    view[:fh,:fw] = bgr
    for d in detections:
        m = d['metrics']
        cv2.drawContours(view, [d['contour']], -1, (255, 0, 255), 1)
        x, y, _, _ = m['bbox_xywh']
        cv2.polylines(view,[np.rint(d['box']).astype(np.int32)],True,(0,255,255),1)
        label = '#{} {}'.format(m.get('track_id',m['id']), m['decision'])
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,0,0), 3)
        cv2.putText(view, label, (x, max(16, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1)
        text = '#{} A={} box={:.3f} model={} gap={:.2f}'.format(
            m.get('track_id',m['id']),m['model_area_pixels'],m['oriented_w_over_h'],
            m['model_label'],m['model_margin'])
        cv2.putText(view,text,(8,fh+24*m['id']),cv2.FONT_HERSHEY_SIMPLEX,.46,(240,240,240),1)
    return view


def write_png(path, pixels):
    ok, encoded = cv2.imencode('.png', pixels)
    if not ok:
        raise OSError('Could not encode PNG')
    encoded.tofile(str(path))


def save_result(output_dir, rgb, out, mask, detections, reference, settings, rules, source=None):
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = output_dir / datetime.now().strftime('texture_%Y%m%d_%H%M%S_%f')
    write_png(Path(str(prefix)+'_raw.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_converted.png'), cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
    write_png(Path(str(prefix)+'_mask.png'), mask)
    write_png(Path(str(prefix)+'_debug.png'), debug_view(out, detections))
    write_png(Path(str(prefix)+'_reference.png'), reference['cutout_bgra'])
    Path(str(prefix)+'.json').write_text(json.dumps({
        'source':source,'settings':asdict(settings),'rules':asdict(rules),
        'image_width':int(rgb.shape[1]),'image_height':int(rgb.shape[0]),
        'program_version':'area_model_320x240_capture',
        'model_status':'Original 640x480-trained normalized-area model; new 320x240 data not yet fitted',
        'model':'102-photo area/rotated-box-ratio RBF; 36 support vectors',
        'targets': [d['metrics'] for d in detections]}, indent=2), encoding='utf-8')
    print('Saved: {}_*'.format(prefix))
    return prefix


def check_capture_size(rgb):
    if rgb.shape != (CAMERA_HEIGHT,CAMERA_WIDTH,3):
        raise ValueError('Expected native 320x240 RGB frame, got {}. Check camera output; no automatic resize is applied.'.format(rgb.shape))


def configure_camera(cam):
    # Exact Recording_N02.py settings/order. No added Picamera2 controls.
    camera = cam.camera
    camera.framerate = 30
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.iso = 100
    camera.shutter_speed = 15000
    camera.exposure_compensation = 0
    camera.awb_gains = (1.2, 1.5)


class LivePreview:
    ORIGINAL = 'Original 320x240'
    CONVERTED = 'Green texture 320x240 | S save | R reference | D debug | C reset | Q exit'
    DEBUG = 'Shape debug'
    MASK = 'Stable red foreground mask'

    def __init__(self, args, reference, settings, rules):
        self.args, self.reference, self.settings, self.rules = args, reference, settings, rules
        self.show_debug = args.debug
        self.tracker = Stabilizer(rules)
        self.started = self.last_print = time.monotonic()
        for title in (self.ORIGINAL, self.CONVERTED):
            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)

    def run(self, rgb):
        now = time.monotonic()
        if rgb is None:
            if now-self.started > 10:
                raise RuntimeError('No camera image after 10 seconds')
            return
        check_capture_size(rgb)
        out, mask, detections = process_frame(rgb,self.reference,self.settings,self.rules,self.tracker)
        cv2.imshow(self.ORIGINAL, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imshow(self.CONVERTED, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
        if self.show_debug:
            cv2.imshow(self.DEBUG, debug_view(out, detections))
            cv2.imshow(self.MASK,self.tracker.last_mask)
        if now-self.last_print >= 1:
            print(' | '.join('#{} A={} box={:.3f} model={} gap={:.2f} state={} votes={}'.format(
                d['metrics'].get('track_id',d['metrics']['id']),
                d['metrics']['model_area_pixels'],d['metrics']['oriented_w_over_h'],
                d['metrics']['model_label'],d['metrics']['model_margin'],d['metrics']['decision'],
                d['metrics']['pending_votes']) for d in detections)
                or 'No red target')
            self.last_print = now
        key = cv2.waitKey(1) & 255
        if key in (ord('s'), ord('S')):
            try:
                save_result(self.args.output_dir, rgb, out, mask, detections,
                            self.reference, self.settings, self.rules)
            except (OSError, cv2.error) as exc:
                print('Save failed: {}'.format(exc), file=sys.stderr)
        elif key in (ord('r'), ord('R')):
            try:
                self.reference = sample_reference(rgb)  # Always original, never converted.
                print('Green reference updated from ORIGINAL frame; held until next R.')
            except ValueError as exc:
                print(str(exc))
        elif key in (ord('d'), ord('D')):
            if self.show_debug:
                cv2.destroyWindow(self.DEBUG)
                cv2.destroyWindow(self.MASK)
            self.show_debug = not self.show_debug
        elif key in (ord('c'),ord('C')):
            self.tracker.reset()
            print('Tracking and class memory cleared')
        elif key in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        if cv2.getWindowProperty(self.CONVERTED, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt

    def shutdown(self):
        cv2.destroyAllWindows()


def read_rgb(path):
    bgr = cv2.imdecode(np.fromfile(str(Path(path).expanduser()), np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise OSError('Cannot read image: {}'.format(path))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def build_parser(default_rules=None):
    defaults = default_rules if default_rules is not None else Rules()
    color_defaults = Settings()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='Photo test instead of camera')
    parser.add_argument('--reference-image', type=Path, help='Photo containing a real green reference pillar')
    parser.add_argument('--no-window', action='store_true', help='Photo test only')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent/'captures_320x240')
    parser.add_argument('--model-margin',type=float,default=defaults.model_margin,help='Model decision margin, not probability')
    parser.add_argument('--edge-pixels',type=int,default=defaults.edge_pixels,help='Dark red edge search radius, 0..6')
    parser.add_argument('--edge-s-min',type=int,default=defaults.edge_s_min)
    parser.add_argument('--edge-v-min',type=int,default=defaults.edge_v_min)
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--switch-frames',type=int,default=defaults.switch_frames,help='Consecutive votes to change a tracked class')
    parser.add_argument('--weak-s-min',type=int,default=color_defaults.weak_s_min,help='Connected dark-red saturation minimum')
    parser.add_argument('--weak-v-min',type=int,default=color_defaults.weak_v_min,help='Connected dark-red brightness minimum')
    parser.add_argument('--grow-pixels',type=int,default=color_defaults.grow_pixels,help='Bounded dark-red growth around current strong seeds')
    return parser


def main():
    parser=build_parser()
    args = parser.parse_args()
    if not 0<=args.model_margin<=3:parser.error('model-margin: 0..3')
    if not 0 <= args.edge_pixels <= 6 or not 0 <= args.edge_s_min <= 255 or not 0 <= args.edge_v_min <= 255:
        parser.error('edge-pixels: 0..6; edge-s-min/edge-v-min: 0..255')
    if not 1<=args.switch_frames<=30:parser.error('switch-frames: 1..30')
    if not 0 <= args.weak_s_min <= 255 or not 0 <= args.weak_v_min <= 255:
        parser.error('weak-s-min/weak-v-min: 0..255')
    if not 0 <= args.grow_pixels <= 12:parser.error('grow-pixels: 0..12')
    settings = Settings(weak_s_min=args.weak_s_min,weak_v_min=args.weak_v_min,grow_pixels=args.grow_pixels)
    rules = Rules(model_margin=args.model_margin,
                  edge_pixels=args.edge_pixels, edge_s_min=args.edge_s_min, edge_v_min=args.edge_v_min,
                  switch_frames=args.switch_frames)
    print('Running file: {}'.format(Path(__file__).resolve()))
    print('CAMERA: native 320x240 RGB; original height; S saves original raw frame.')
    print('MODEL: previous model retained temporarily; collect new 320x240 raw photos for refitting.')
    print('Program arguments: {}'.format(sys.argv[1:]))
    print('ACTIVE: learned AREA+BOX model, model_margin={:.3f}, switch_frames={}'.format(
        rules.model_margin,rules.switch_frames))
    print('No normal_min/normal_max ratio thresholds are used in this version.')
    reference = sample_reference(read_rgb(args.reference_image)) if args.reference_image else built_in_reference()
    print('Reference: {}'.format(args.reference_image or 'embedded real green pillar from approved photo'))
    if args.image:
        rgb = read_rgb(args.image)
        check_capture_size(rgb)
        out,mask,ds = process_frame(rgb,reference,settings,rules,Stabilizer(rules))
        save_result(args.output_dir, rgb, out, mask, ds, reference, settings, rules, str(args.image))
        if not args.no_window:
            try:
                cv2.imshow('Photo result', cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
                cv2.waitKey(0)
            finally:
                cv2.destroyAllWindows()
        return
    if args.no_window:
        parser.error('--no-window is only available with --image')
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
        parser.error('Use Raspberry Pi desktop/VNC for live preview; DISPLAY is not set')
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    car = dk.vehicle.Vehicle()
    cam = PiCamera(image_w=CAMERA_WIDTH, image_h=CAMERA_HEIGHT, image_d=3)
    try:
        time.sleep(2)
        configure_camera(cam)
        preview = LivePreview(args, reference, settings, rules)
    except BaseException:
        cam.shutdown()
        raise
    car.add(cam, outputs=['cam/image_array'], threaded=True)
    car.add(preview, inputs=['cam/image_array'], outputs=[])
    car.start(rate_hz=20)


if __name__ == '__main__':
    def stop(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError, ImportError, cv2.error) as exc:
        print('ERROR: {}'.format(exc), file=sys.stderr)
        sys.exit(1)
