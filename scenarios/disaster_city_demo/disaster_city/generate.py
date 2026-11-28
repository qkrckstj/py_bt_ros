"""Generate a static R2025a ENU world and a matching 5 cm occupancy map."""
import json
import random
import sys
from pathlib import Path
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parent
seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1128
rng = random.Random(seed)
solids = []
def box(name,x,y,w,d,h,color):
    solids.append((name,x,y,w,d,h,color))

for x in (-12, -4, 4, 12):
    for y in (-8, 0, 8):
        box(f'block{len(solids)}',x+rng.uniform(-.6,.6),y+rng.uniform(-.6,.6),
            rng.uniform(3.1,4.9),rng.uniform(3.1,5.0),rng.uniform(1.8,3.6),
            rng.choice(['0.52 0.6 0.65','0.65 0.55 0.46','0.58 0.64 0.57']))
for x in (-17.9,17.9):
    box(f'wall{len(solids)}',x,0,.2,28,.5,'0.4 0.4 0.4')
for y in (-13.9,13.9):
    box(f'wall{len(solids)}',0,y,36,.2,.5,'0.4 0.4 0.4')

def solid(name,x,y,z,w,d,h,color):
    return f'''Solid {{ name "{name}" translation {x} {y} {z}
 children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
 geometry Box {{ size {w} {d} {h} }} }} ] boundingObject Box {{ size {w} {d} {h} }} }}
'''

world = '''#VRML_SIM R2025a utf8
WorldInfo { coordinateSystem "ENU" basicTimeStep 20 }
Viewpoint { orientation -0.577350269 0.577350269 0.577350269 2.094395102 position 0 0 55 }
Background { skyColor [ 0.77 0.85 0.9 ] }
DirectionalLight { direction -0.3 -0.2 -1 castShadows FALSE }
''' + solid('ground',0,0,-.06,36,28,.12,'0.32 0.35 0.37')
# Webots Viewpoint forward axis is -Z at identity.
im=Image.new('L',(720,560),254)
draw=ImageDraw.Draw(im)
for name,x,y,w,d,h,c in solids:
    world += solid(name,x,y,h/2,w,d,h,c)
    draw.rectangle(((x-w/2+18)*20,(14-y-d/2)*20,(x+w/2+18)*20,(14-y+d/2)*20), fill=0)
(OUT/'city.wbt').write_text(world)
im.save(OUT/'city.pgm')
(OUT/'city.yaml').write_text('image: city.pgm\nmode: trinary\nresolution: 0.05\norigin: [-18.0, -14.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

starts=[[x,y] for y in (-11.4,-12.8) for x in (-16,-14,-12)]
sites=[(x,y) for x in (-12,-8,-4,0,4,8,12) for y in (-4,4,12)]
picks=rng.sample(sites,7)
catalog=[('fire',2,4,30),('power_box',2,3,20),('hydrant',2,4,20),
         ('small_debris',1,3,10),('large_debris',4,6,50),('fallen_tree',3,5,30),('cordon',1,2,10)]
import math
tasks=[]
for (x,y),(kind,low,high,secs) in zip(picks,catalog):
    poses=[[round(x+.95*math.cos(2*math.pi*i/high),3),round(y+.95*math.sin(2*math.pi*i/high),3)] for i in range(high)]
    tasks.append(dict(kind=kind,xy=[x,y],minimum=low,maximum=high,seconds=secs,work_positions=poses))
charge=[-16,-9]
(OUT/'scenario.json').write_text(json.dumps(dict(seed=seed,starts=starts,charge_xy=charge,slots=1,survey=sites,tasks=tasks, initial_battery=[28,52,66,78,88,96], speed_mps=0.2, battery_rates=dict(idle=0.015,move=0.07,work=0.14,charge=0.8), reserve=8, discovery_radius=1.5, horizon_s=3600),indent=2))
# Connected free space with 0.35 m clearance, at 5 cm resolution.
from PIL import ImageFilter
free=im.filter(ImageFilter.MinFilter(15))
mask=free.copy()
def px(pos):
    x,y=pos
    return int((x+18)*20),int((14-y)*20)
ImageDraw.floodfill(mask,px(starts[0]),128)
for p in starts+list(sites)+[charge]+[p for t in tasks for p in t['work_positions']]:
    assert mask.getpixel(px(p))==128, p
im.convert('RGB').save(OUT/'city.png')
print(seed, len(tasks), 'tasks; all starts, survey, charging and docking positions connected')

(OUT/'geometry.json').write_text(json.dumps(solids,indent=2))
