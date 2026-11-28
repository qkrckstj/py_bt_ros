"""Known occupancy only. No hidden task information enters route costs."""
from collections import deque
from pathlib import Path
import yaml
import math
from PIL import Image, ImageFilter


class CityGrid:
    def __init__(self, path):
        self.image = Image.open(path).convert('L')
        meta=yaml.safe_load(Path(path).with_suffix('.yaml').read_text())
        self.resolution=meta['resolution'];self.origin=meta['origin'][:2]
        self.top=self.origin[1]+self.image.height*self.resolution
        self.width=round(self.image.width*self.resolution/.25)
        self.height=round(self.image.height*self.resolution/.25)
        self.free = self.image.filter(ImageFilter.MinFilter(15)).resize((self.width,self.height), Image.Resampling.NEAREST)
        self.cache = {}

    def cell(self, xy):
        return (max(0,min(self.width-1,int((xy[0]-self.origin[0])/.25))),max(0,min(self.height-1,int((self.top-xy[1])/.25))))

    def visible(self, a, b):
        count=max(2,math.ceil(math.dist(a,b)/.04))
        for i in range(count+1):
            x=a[0]+(b[0]-a[0])*i/count; y=a[1]+(b[1]-a[1])*i/count
            px,py=int((x-self.origin[0])/self.resolution),int((self.top-y)/self.resolution)
            if not (0<=px<self.image.width and 0<=py<self.image.height) or self.image.getpixel((px,py))<200:return False
        return True

    def anchor(self,pos):
        cell=self.cell(pos)
        if self.free.getpixel(cell)>200:return cell,0.
        candidates=[]
        for x in range(max(0,cell[0]-3),min(self.width,cell[0]+4)):
            for y in range(max(0,cell[1]-3),min(self.height,cell[1]+4)):
                center=(self.origin[0]+(x+.5)*.25,self.top-(y+.5)*.25)
                d=math.dist(pos,center)
                if d<=.6 and self.free.getpixel((x,y))>200 and self.visible(pos,center):
                    candidates.append((d,(x,y)))
        if not candidates:return None,math.inf
        d,cell=min(candidates)
        return cell,d

    def distance(self, a, b):
        source,extra_a=self.anchor(a);target,extra_b=self.anchor(b)
        if source is None or target is None:return math.inf
        if target not in self.cache:
            distances={target:0.}; q=deque([target])
            while q:
                x,y=q.popleft()
                for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if 0<=p[0]<self.width and 0<=p[1]<self.height and p not in distances and self.free.getpixel(p)>200:
                        distances[p]=distances[(x,y)]+.25;q.append(p)
            self.cache[target]=distances
        return self.cache[target].get(source,math.inf)+extra_a+extra_b

    def center(self,cell):
        return (self.origin[0]+(cell[0]+.5)*.25,self.top-(cell[1]+.5)*.25)

    def path(self,a,b):
        if not math.isfinite(self.distance(a,b)):return []
        source,_=self.anchor(a);target,_=self.anchor(b);dist=self.cache[target]
        cells=[source]
        while cells[-1]!=target:
            x,y=cells[-1]
            choices=[p for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)) if dist.get(p,math.inf)<dist[cells[-1]]]
            if not choices:return []
            cells.append(min(choices,key=lambda p:dist[p]))
        return [list(a)]+[list(self.center(c)) for c in cells]+[list(b)]
