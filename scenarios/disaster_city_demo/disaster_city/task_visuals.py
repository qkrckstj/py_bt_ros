"""Presentation panels derived solely from the public task snapshot."""
from PIL import Image, ImageDraw, ImageFont

NAMES = {'fire':'FIRE', 'power_box':'POWER BOX', 'hydrant':'HYDRANT',
         'small_debris':'SMALL DEBRIS', 'large_debris':'LARGE DEBRIS',
         'fallen_tree':'FALLEN TREE', 'cordon':'CORDON'}


def panel_values(task):
    percent = max(0, min(100, int(100 * task['progress_s'] / task['seconds'])))
    crew = task['working'] if task['state'] == 'working' else task['arrived']
    rate = (1 + .5 * (task['working'] - task['minimum'])
            if task['state'] == 'working' and task['working'] >= task['minimum'] else 0.)
    return percent, crew, rate, task['state'], task['assigned']


def render_panel(task, path):
    percent, crew, rate, state, assigned = panel_values(task)
    im = Image.new('RGBA', (640, 220), (0,0,0,0)); d = ImageDraw.Draw(im)
    regular='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    bold='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
    title=ImageFont.truetype(bold,30); body=ImageFont.truetype(regular,27)
    d.rounded_rectangle((1,1,638,218),radius=18,fill=(30,37,43,235),outline=(160,170,177,255),width=2)
    accent=(132,192,159,255) if state in ('working','done') else (216,183,121,255)
    d.text((20,14),NAMES.get(task['kind'],task['kind']),font=title,fill='white')
    d.text((620,14),f'{percent}%',font=title,fill='white',anchor='ra')
    d.rounded_rectangle((20,62,620,91),radius=8,fill=(69,77,84,255))
    if percent:d.rounded_rectangle((20,62,20+6*percent,91),radius=7,fill=accent)
    status='COMPLETE' if state=='done' else f'{rate:.1f}x'
    d.text((20,107),f'Crew {crew} / {task["minimum"]} required',font=body,fill='white')
    d.text((620,107),status,font=title,fill=accent,anchor='ra')
    detail=('Task finished' if state=='done' else
            f'Working | assigned {assigned} | max {task["maximum"]}' if rate else
            f'Waiting for crew | assigned {assigned}')
    d.text((20,164),detail,font=body,fill=(190,200,207,255))
    # Webots textures use power-of-two dimensions; UV mapping preserves panel aspect.
    im.resize((1024,256), Image.Resampling.LANCZOS).save(path)
