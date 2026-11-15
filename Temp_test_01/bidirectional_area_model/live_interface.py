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
