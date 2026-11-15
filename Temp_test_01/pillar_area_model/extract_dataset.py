"""Recompute features from original labeled photos using the deployment detector."""
from pathlib import Path
import csv
import json
import sys
import numpy as np
import cv2
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'green_texture_live'))
import green_texture_live_thonny as g


def main():
    rows,details = [],[]
    for label in ['tall','stand','low']:
        folder=Path(r'C:\Users\xiaoy\Desktop')/label
        tiles=[];counts={}
        for index,path in enumerate(sorted(folder.glob('*_raw.png'))):
            rgb=g.read_rgb(path);mask,ds=g.detect_red(rgb,g.Settings())
            counts[len(ds)]=counts.get(len(ds),0)+1
            if not ds:
                print('NO TARGET:',path);continue
            d=max(ds,key=lambda d:d['metrics']['contour_area_px'])
            m=d['metrics'];m.update(g.average_edge_geometry(d))
            region=np.zeros(mask.shape,np.uint8)
            cv2.drawContours(region,[d['foreground_contour']],-1,255,-1)
            area=int(cv2.countNonZero(region));x,y,w,h=m['bbox_xywh']
            row={'label':label,'index':index,'file':str(path),'target_count':len(ds),
                 'area_pixels':area,'area_fraction':area/region.size,'log_area':float(np.log(area/region.size)),
                 'avg_ratio':m.get('average_w_over_h'),'box_ratio':m['oriented_w_over_h'],
                 'contour_area':m['contour_area_px'],'quad_valid':m['average_quad_valid'],
                 'fill':m['rectangle_fill'],'quad_fill':m.get('quad_fit_fraction'),
                 'bbox_x':x,'bbox_y':y,'bbox_w':w,'bbox_h':h,
                 'center_x':(x+w/2)/region.shape[1],'center_y':(y+h/2)/region.shape[0],
                 'partial':m['touches_image_border'],
                 'top_bottom_ratio':m.get('edge_top_width_px',1)/max(m.get('edge_bottom_width_px',1),1),
                 'left_right_ratio':m.get('edge_left_height_px',1)/max(m.get('edge_right_height_px',1),1)}
            rows.append(row);details.append({'row':row,'metrics':m})
            tile=Image.new('RGB',(260,245),(238,238,238))
            im=Image.fromarray(rgb);im.thumbnail((260,195));tile.paste(im,(0,20))
            draw=ImageDraw.Draw(tile)
            draw.text((5,3),f'{label} {index:02d} n={len(ds)} A={area}',fill='black')
            draw.text((5,216),f'avg={row["avg_ratio"]:.3f} box={row["box_ratio"]:.3f}'+(' CLIP' if row['partial'] else ''),fill='black')
            draw.rectangle((x*260/rgb.shape[1],20+y*195/rgb.shape[0],
                            (x+w)*260/rgb.shape[1],20+(y+h)*195/rgb.shape[0]),outline='yellow')
            tiles.append(tile)
        sheet=Image.new('RGB',(260*6,245*((len(tiles)+5)//6)),(220,220,220))
        for i,tile in enumerate(tiles):sheet.paste(tile,((i%6)*260,(i//6)*245))
        sheet.save(ROOT/(label+'_contact_sheet.jpg'))
        print(label,'target counts',counts)
    with (ROOT/'measurements.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    (ROOT/'measurements.json').write_text(json.dumps(details,indent=2),encoding='utf-8')
    for label in ['tall','stand','low']:
        rr=[r for r in rows if r['label']==label]
        print(label,len(rr),'area',min(r['area_pixels'] for r in rr),max(r['area_pixels'] for r in rr),
              'avg',min(r['avg_ratio'] for r in rr),max(r['avg_ratio'] for r in rr),
              'clipped',sum(r['partial'] for r in rr))


if __name__=='__main__':main()
