from pathlib import Path
import ast,json
root=Path('bidirectional_area_model');s=Path('pillar_area_model/green_texture_live_area_model_320x240.py').read_text(encoding='utf-8');lines=s.splitlines();tree=ast.parse(s)
keep={'Settings','Rules','seeded_red_mask','detect_red','ordered_corners','average_edge_geometry','Stabilizer','write_png','configure_camera','read_rgb'}
pieces=[]
for node in tree.body:
 if isinstance(node,(ast.Import,ast.ImportFrom)) or (isinstance(node,(ast.ClassDef,ast.FunctionDef)) and node.name in keep):
  start=min([node.lineno]+[d.lineno for d in getattr(node,'decorator_list',[])])-1
  pieces.append('\n'.join(lines[start:node.end_lineno]))
base='\n\n'.join(pieces)
base=base.replace('min_area: int = 150','min_area: int = 30').replace('min_size: int = 12','min_size: int = 5').replace('grow_pixels: int = 4','grow_pixels: int = 2').replace('edge_pixels: int = 2','edge_pixels: int = 1')
base=base.replace('switch_frames: int = 5','switch_frames: int = 5\n    straight_edges: bool = True')
models=json.loads((root/'models.json').read_text());palettes=json.loads((root/'palettes.json').read_text())
for model in models.values():model['default_margin']=.15
(root/'models.json').write_text(json.dumps(models,indent=2),encoding='utf-8')
payload='\nMODELS = json.loads('+repr(json.dumps(models))+')\nPALETTES = json.loads('+repr(json.dumps(palettes))+')\n'
tail='''
if __name__=='__main__':
    def stop(_signum,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:main()
    except KeyboardInterrupt:pass
    except (OSError,ValueError,RuntimeError,ImportError,cv2.error) as exc:
        print('ERROR:',exc,file=sys.stderr);sys.exit(1)
'''
program='#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n"""320x240 bidirectional nonstandard-pillar color replacement, no height change."""\n'+base+'\n'+payload+'\n'+(root/'color_engine.py').read_text(encoding='utf-8')+'\n'+(root/'live_interface.py').read_text(encoding='utf-8-sig')+'\n'+tail
(root/'green_red_live_area_model_320x240.py').write_text(program,encoding='utf-8')
