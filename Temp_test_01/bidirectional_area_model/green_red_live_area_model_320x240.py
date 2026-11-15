#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""320x240 bidirectional nonstandard-pillar color replacement, no height change."""
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

@dataclass
class Settings:
    # Same red HSV defaults as Recording_N02.py, including the hue wraparound.
    hue_width: int = 10
    s_min: int = 100
    v_min: int = 70
    min_area: int = 30
    min_size: int = 5
    kernel: int = 3
    weak_s_min: int = 55
    weak_v_min: int = 10
    grow_pixels: int = 2

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
    edge_pixels: int = 1
    edge_s_min: int = 40
    edge_v_min: int = 15
    switch_frames: int = 5
    straight_edges: bool = True

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

def write_png(path, pixels):
    ok, encoded = cv2.imencode('.png', pixels)
    if not ok:
        raise OSError('Could not encode PNG')
    encoded.tofile(str(path))

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

def read_rgb(path):
    bgr = cv2.imdecode(np.fromfile(str(Path(path).expanduser()), np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise OSError('Cannot read image: {}'.format(path))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

MODELS = json.loads('{"red": {"features": ["log_area", "box_ratio"], "classes": ["low", "stand", "tall"], "mean": [-4.59934984041926, 0.5884339822642841], "scale": [1.178122299649499, 0.33019868746017955], "support_vectors": [[0.973812535768435, 2.222262899451362], [-0.5823085899504299, 0.23692601355908713], [0.26536581613191956, 0.5916128953209663], [1.075298194238959, 1.0870254551914926], [1.1589516776297426, 1.532074839867065], [-0.4796706299587555, 1.7006912476096323], [-1.602071486534357, 0.2369260913910114], [0.1381308337083427, 1.8601638640369713], [-0.6457157466673425, 0.3231035957231149], [0.6457199047423564, 0.9025270452027533], [-1.449737142413257, 1.2464193025762673], [-1.3799327611011987, 0.2575259735213547], [-0.9747306103223926, -0.6134625231470928], [-0.3278197995571267, -0.30243106903600064], [0.1125366842896796, -0.16061751017033424], [0.49003905617315324, -0.02065201536625754], [-0.8954067836832436, -0.3336570279745678], [-0.19351794883927942, -0.49023366270764446], [0.40987890059224164, -0.3838721365298248], [0.9275708841301046, 0.014096849759185154], [1.3529456404364506, 0.4594618930258619], [1.5773748887408992, 0.8164701422828385], [0.4918876412484549, -0.34194359671179175], [1.4745893626566404, 0.34448993376688664], [1.3872831589880494, 0.17434050852704544], [-0.3262045596566072, -0.5316302357653826], [-0.8433914443281998, -0.6071692579000613], [0.7495092320508554, 7.574126807807371], [1.3396739535025657, 0.2192156735456713], [-0.6575542692765823, -0.5838147006884744], [-1.1136976181552098, -0.7286761106122768], [-1.1555192940724226, -0.6633050903087828], [-0.9113323598828496, -0.6215100348966806], [-0.5955027671953771, -0.5364136572432473], [-0.19905668504422536, -0.5175159392580844], [0.22756171511103873, -0.39400716559977345], [0.373871844366778, -0.44952885381950874], [-0.17711534544899338, -0.5103154132175581], [-0.6066567819907579, -0.6716177986757188], [-0.24179096002965114, -0.48413954835784895], [1.458581003728425, 0.3370820235929851], [1.403208735187655, 0.10341183597487258], [-0.5071807145742463, -0.6007392154353013], [-0.7887961227671918, -0.6471042670871313], [1.6332366043492077, 2.524269819204356], [0.8342012099909643, 0.13280759528515867], [-0.5294391805305427, -0.523650822036709], [-0.8433914443281998, -0.5879461511615237], [-1.0622389352710384, -0.5175369149699094], [-0.6156867428091047, -0.7488138134858251], [-0.5091802639361075, -0.7989017697048387], [-0.41456031955827616, -0.7460006266755674], [-0.19766860900049213, -0.7829733927640655], [2.2824757278701986, 0.4293998691948411], [1.663647282246343, -0.14331375899212612], [-0.6671454256932764, -0.8174866080220354], [-0.6433677277106409, -0.7713763998462838], [-0.6433677277106409, -0.7811225180865027], [-0.681738478167954, -0.8421871592175428], [-0.6965868156051807, -0.8706670106418014], [-0.6599419172970401, -0.8155255954539575], [-0.6575542692765823, -0.8356604454933229], [-0.6457157466673425, -0.7410204569447051], [-0.6866591198416158, -0.8582792304388954], [0.0721330725016635, -0.7098656095215619], [0.4173039113004252, -0.6182100351787444], [0.854518319543895, -0.4884279295006265], [1.6155676562372252, -0.1636111034158376], [1.1152067899176064, 6.427502632243466], [0.23675618357376393, 1.316218669348747], [1.4117900290325929, -0.29021262344388354], [1.1572669764913497, -0.3710869343244968], [0.9497513310310978, -0.44321146479506], [0.6207071685725418, -0.5419842832224645], [0.4125864254129238, -0.6123782213872], [-0.18527702671273139, -0.7087812108661423], [2.178646788793357, 0.4302333020435075], [2.508800295850098, 0.44737412786971775], [-0.8170923533686408, -0.9303004340419406]], "dual_coef": [[1.0, 1.0, 1.0, 1.0, 0.4519015487134519, 0.4532748487516953, 1.0, 0.0, 0.804797941344713, 0.33638537700094495, 0.33201841164226986, 0.13774642098301473, -0.0, -0.2332831246808874, -1.0, -0.5961215337298211, -1.0, -0.0, -0.0, -0.0, -0.7548899392771904, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.8430918209997691, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.24100977197302667, -0.0, -0.0, -0.8477283577753955, -0.36413920584097986, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.34803379227848225, -1.0, -1.0, -0.0, -0.26645112249144626, -0.0, -0.7771055691211426, -0.0, -0.0, -0.5270451807001285, -0.4822501997411937, -0.0], [0.6293017848430345, 0.9774297853552301, 0.8720823896697544, 1.0, 0.0, 0.5791278613853387, 0.6805773877873618, 0.6129779451674512, 0.0, 0.0, 0.41352791596520316, 0.0, 0.033517682915945125, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.6210193662360999, 0.0, 0.17815736091956846, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.3588339524032785, 1.0, 1.0, 1.0, 1.0, 0.0, 0.5470044734324895, 1.0, 0.0, -1.0, -1.0, -1.0, -0.864604035383522, -0.695639987383303, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -0.7102495192152019, -1.0, -1.0, -1.0, -0.28655722445663157, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -0.0, -0.18148206946872286]], "intercept": [-0.15648933529939063, 0.0021099396377592546, -0.1428641133907259], "n_support": [12, 37, 30], "gamma": 1.0, "area_range": [0.0013411458333333333, 0.19326822916666667], "training_samples": 238, "selection": {"color": "red", "scope": "pooled", "feature_set": "area_box", "features": ["log_area", "box_ratio"], "C": 1, "gamma": 1, "n": 294, "accuracy": 1.0, "coverage": 1.0, "accepted_accuracy": 1.0, "standard_false_conversion": 0.0, "special_missed_conversion": 0.0, "confusion": [[117, 0, 0], [0, 105, 0], [0, 0, 72]], "selection_loss": 0.0}, "default_margin": 0.15}, "green": {"features": ["log_area", "full_box_ratio"], "classes": ["low", "stand", "tall"], "mean": [-4.59934984041926, 0.5872415344254309], "scale": [1.178122299649499, 0.26908855399722237], "support_vectors": [[0.973812535768435, 2.9839088417387853], [1.075298194238959, 1.5767930847521223], [0.6457199047423564, 1.1100360814919823], [-1.1343509074332159, 0.22229600349902565], [1.5773748887408992, 0.9977375717632856], [1.3872831589880494, 0.22431835041313827], [-1.035770607283388, -0.7887423350887743], [-0.9993354725526351, -0.41358845455196386], [0.7495092320508554, 4.4007336359586775], [-0.6575542692765823, -0.7344471935699441], [-0.5955027671953771, -0.7141881727661983], [-0.19905668504422536, -0.6590261687448072], [0.22756171511103873, -0.5471861757863421], [0.373871844366778, -0.5023591840671052], [-0.17711534544899338, -0.6371585736485875], [1.458581003728425, 0.3585293101394903], [-0.5071807145742463, -0.7183587925769727], [-0.7887961227671918, -0.7856644028275419], [1.6332366043492077, 3.035427520235168], [0.8342012099909643, 0.15105623075759603], [-0.6156867428091047, -0.8978276833348732], [-0.41456031955827616, -0.891593295046685], [2.2824757278701986, 0.5332277436886519], [1.663647282246343, -0.13910935135995348], [-0.6433677277106409, -0.9315002024662209], [-0.6433677277106409, -0.9375721163336531], [-0.6866591198416158, -0.9324119198702174], [-0.6386910860643457, -0.9222291927272585], [-0.4400082991278467, -0.894793830705175], [-0.21024377695168775, -0.8517391059804416], [0.854518319543895, -0.5813550891602447], [1.1152067899176064, 0.6788471322720091], [0.23675618357376393, 3.840550185920058], [0.9497513310310978, -0.51019971008522], [0.6207071685725418, -0.6324313100486385]], "dual_coef": [[7.274838166615897, 3.5753639429841737, 0.0, 7.872153544247703, -3.3346139011334315, -0.0, -0.0, -7.204856037260177, -1.29453008538893, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -6.802198666508374, -0.08615696355686131, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.523362158769048, -0.0, -0.0, -0.0, -0.0, -7.289845380624849, -2.3865923460055805, -0.0, -0.0], [2.1785943116591295, 0.0, 7.234303310981342, 1.7869022627590063, 0.7080308996560958, 10.0, 10.0, 0.0, 6.11810756264857, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 2.0552311218817123, 0.0, -10.0, -10.0, -4.375486873437463, -4.466122239265897, -10.0, -3.1697084463829577, -10.0, -10.0, -10.0, -10.0, -0.48009923767598583, -10.0, -6.389952787424076, -10.0, -10.0]], "intercept": [-0.7965354609808448, -0.5637654955733953, -0.6272409770409916], "n_support": [4, 16, 15], "gamma": 0.3, "area_range": [0.0013411458333333333, 0.19326822916666667], "training_samples": 238, "selection": {"color": "green", "scope": "pooled", "feature_set": "area_full_box", "features": ["log_area", "full_box_ratio"], "C": 10, "gamma": 0.3, "n": 420, "accuracy": 0.9738095238095238, "coverage": 1.0, "accepted_accuracy": 0.9738095238095238, "standard_false_conversion": 0.036231884057971016, "special_missed_conversion": 0.010638297872340425, "confusion": [[147, 3, 3], [2, 133, 3], [0, 0, 129]], "selection_loss": 0.09095920884542533}, "default_margin": 0.15}}')
PALETTES = json.loads('{"red": {"median_hsv": [177.0, 232.0, 159.0], "standard_photos_used": 34, "example_source": "C:\\\\Users\\\\xiaoy\\\\Desktop\\\\captures_320x240_standard_red\\\\texture_20261115_143933_832745_raw.png", "example_bbox": [226, 36, 46, 53]}, "green": {"median_hsv": [76.0, 213.0, 88.5], "standard_photos_used": 43, "example_source": "C:\\\\Users\\\\xiaoy\\\\Desktop\\\\captures_320x240_standard_green\\\\texture_20261115_144408_541454_raw.png", "example_bbox": [205, 38, 43, 54]}}')

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

"""UI and saving for the standalone bidirectional Raspberry Pi program."""

def save_result(output_dir,rgb,out,mask,detections,reference,settings,rules,source=None):
    output_dir=Path(output_dir).expanduser();output_dir.mkdir(parents=True,exist_ok=True)
    prefix=output_dir/datetime.now().strftime('dual_%Y%m%d_%H%M%S_%f')
    for suffix,pixels in [('raw',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)),('converted',cv2.cvtColor(out,cv2.COLOR_RGB2BGR)),
                           ('mask',mask),('debug',debug_view(out,detections)),
                           ('red_mask',process_frame.last_color_masks['red']),('green_mask',process_frame.last_color_masks['green'])]:
        write_png(Path(str(prefix)+'_'+suffix+'.png'),pixels)
    Path(str(prefix)+'.json').write_text(json.dumps({'source':source,'program_version':'bidirectional_area_320x240',
        'width':320,'height':240,'settings':asdict(settings),'rules':asdict(rules),'palettes':reference or PALETTES,
        'frame_processing_ms':process_frame.last_timing_ms,
        'targets':[d['metrics'] for d in detections]},indent=2),encoding='utf-8')
    print('Saved {}_*'.format(prefix));return prefix


def debug_view(rgb,detections):
    view=np.full((240+30+len(detections)*34,640,3),25,np.uint8)
    view[:240,:320]=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
    for index,d in enumerate(detections):
        m=d['metrics'];x,y,w,h=m['bbox_xywh'];color=(255,80,255) if m['source_color']=='red' else (0,200,255)
        cv2.drawContours(view,[d['foreground_contour']],-1,color,1)
        cv2.putText(view,'#{} {}'.format(m['id'],m['model_label']),(x,max(y-3,12)),0,.32,color,1)
        text='#{} {} -> {} {} gap={:.2f}'.format(m['id'],m['source_color'],m['target_color'],m['decision'],m['model_margin'])
        cv2.putText(view,text,(5,258+34*index),0,.40,(235,235,235),1)
        text='A={} ({:.4f}) box={:.3f} avg={:.3f} fullavg={:.3f}'.format(m['model_area_pixels'],m['model_area_fraction'],m['box_ratio'],m['avg_ratio'],m['full_avg_ratio'])
        cv2.putText(view,text,(5,273+34*index),0,.36,(235,235,235),1)
    return view


class DualPreview:
    ORIGINAL='Original 320x240'
    RESULT='Bidirectional color | S save | D debug | C reset | Q exit'
    DEBUG='Area / average aspect debug'
    RED='Red mask 320x240';GREEN='Green mask 320x240'
    def __init__(self,args,settings,rules):
        self.args,self.settings,self.rules=args,settings,rules
        self.tracker={c:Stabilizer(rules) for c in ['red','green']}
        self.show_debug=args.debug;self.started=self.last_print=time.monotonic();self.times=deque(maxlen=120)
        for name in [self.ORIGINAL,self.RESULT]:cv2.namedWindow(name,cv2.WINDOW_AUTOSIZE)
    def run(self,rgb):
        now=time.monotonic()
        if rgb is None:
            if now-self.started>10:raise RuntimeError('No image after 10 seconds')
            return
        out,mask,ds=process_frame(rgb,PALETTES,self.settings,self.rules,self.tracker)
        self.times.append(process_frame.last_timing_ms)
        cv2.imshow(self.ORIGINAL,cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));cv2.imshow(self.RESULT,cv2.cvtColor(out,cv2.COLOR_RGB2BGR))
        if self.show_debug:
            cv2.imshow(self.DEBUG,debug_view(out,ds))
            cv2.imshow(self.RED,process_frame.last_color_masks['red']);cv2.imshow(self.GREEN,process_frame.last_color_masks['green'])
        if now-self.last_print>=1:
            print(' | '.join('{} A={} avg={:.3f} {} -> {} {} gap={:.2f}'.format(d['metrics']['id'],d['metrics']['model_area_pixels'],
                d['metrics']['avg_ratio'],d['metrics']['source_color'],d['metrics']['target_color'],d['metrics']['decision'],d['metrics']['model_margin']) for d in ds) or 'No color target')
            print('Processing {:.1f} ms | median {:.1f} ms | p95 {:.1f} ms (capture/display/save excluded)'.format(self.times[-1],np.median(self.times),np.percentile(self.times,95)))
            self.last_print=now
        key=cv2.waitKey(1)&255
        if key in [ord('s'),ord('S')]:
            try:save_result(self.args.output_dir,rgb,out,mask,ds,PALETTES,self.settings,self.rules)
            except (OSError,cv2.error) as exc:print('Save failed:',exc)
        elif key in [ord('c'),ord('C')]:
            for tr in self.tracker.values():tr.reset()
            print('Both color trackers cleared')
        elif key in [ord('d'),ord('D')]:
            if self.show_debug:
                for name in [self.DEBUG,self.RED,self.GREEN]:cv2.destroyWindow(name)
            self.show_debug=not self.show_debug
        elif key in [ord('q'),ord('Q'),27]:raise KeyboardInterrupt
        if cv2.getWindowProperty(self.RESULT,cv2.WND_PROP_VISIBLE)<1:raise KeyboardInterrupt
    def shutdown(self):cv2.destroyAllWindows()


def build_parser(default_rules=None):
    rules=default_rules or Rules();settings=Settings()
    p=argparse.ArgumentParser(description='320x240 bidirectional special-pillar color replacement. No height change.')
    p.add_argument('--image',type=Path);p.add_argument('--no-window',action='store_true');p.add_argument('--debug',action='store_true')
    p.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent/'captures_bidirectional_320x240')
    p.add_argument('--model-margin',type=float,default=rules.model_margin)
    p.add_argument('--switch-frames',type=int,default=rules.switch_frames)
    p.add_argument('--min-area',type=int,default=settings.min_area);p.add_argument('--min-size',type=int,default=settings.min_size)
    p.add_argument('--grow-pixels',type=int,default=settings.grow_pixels)
    p.add_argument('--no-straight-edges',action='store_true',default=not rules.straight_edges)
    return p


def main():
    p=build_parser();args=p.parse_args()
    if not 0<=args.model_margin<=3:p.error('model-margin must be 0..3')
    if not 1<=args.switch_frames<=30:p.error('switch-frames must be 1..30')
    if not 1<=args.min_area<=10000 or not 2<=args.min_size<=240:p.error('invalid min-area or min-size')
    if not 0<=args.grow_pixels<=12:p.error('grow-pixels must be 0..12')
    rules=Rules(model_margin=args.model_margin,switch_frames=args.switch_frames,straight_edges=not args.no_straight_edges)
    settings=Settings(min_area=args.min_area,min_size=args.min_size,grow_pixels=args.grow_pixels)
    print('Running file:',Path(__file__).resolve());print('ACTIVE 320x240 RGB; no height changes; special RED <-> GREEN; standard colors retained')
    print('Rules:',asdict(rules));print('Color detection:',asdict(settings))
    for c in ['red','green']:print(c,'features:',MODELS[c]['features'],'standard reference HSV:',PALETTES[c]['median_hsv'])
    if args.image:
        rgb=read_rgb(args.image);out,mask,ds=process_frame(rgb,PALETTES,settings,rules,{c:Stabilizer(rules) for c in ['red','green']})
        save_result(args.output_dir,rgb,out,mask,ds,PALETTES,settings,rules,str(args.image))
        if not args.no_window:
            try:
                cv2.imshow('Photo result',cv2.cvtColor(out,cv2.COLOR_RGB2BGR));cv2.waitKey(0)
            finally:cv2.destroyAllWindows()
        return
    if args.no_window:p.error('--no-window requires --image')
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):p.error('Open Raspberry Pi desktop/VNC to view camera')
    import donkeycar as dk
    from donkeycar.parts.camera import PiCamera
    car=dk.vehicle.Vehicle();cam=PiCamera(image_w=320,image_h=240,image_d=3)
    try:
        time.sleep(2);configure_camera(cam);preview=DualPreview(args,settings,rules)
    except BaseException:
        cam.shutdown();raise
    car.add(cam,outputs=['cam/image_array'],threaded=True);car.add(preview,inputs=['cam/image_array'],outputs=[])
    car.start(rate_hz=20)


if __name__=='__main__':
    def stop(_signum,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:main()
    except KeyboardInterrupt:pass
    except (OSError,ValueError,RuntimeError,ImportError,cv2.error) as exc:
        print('ERROR:',exc,file=sys.stderr);sys.exit(1)
