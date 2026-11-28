"""Bounded read-only map/pose transport. No remote command messages."""
import base64,json,math,zlib
MAX_CELLS=2_000_000
MAX_PACKET=60_000

def encode_map(msg):
    n=msg.info.width*msg.info.height
    if n<1 or n>MAX_CELLS or len(msg.data)!=n:raise ValueError('Invalid map dimensions')
    q=msg.info.origin.orientation;p=msg.info.origin.position
    return {'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
            'origin':[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
            'cells':base64.b64encode(zlib.compress(bytes(v+1 for v in msg.data))).decode()}

def decode_map(data):
    w,h=data['width'],data['height']
    if type(w)!=int or type(h)!=int or w<=0 or h<=0 or w*h>MAX_CELLS:raise ValueError('Invalid dimensions')
    resolution=float(data['resolution']);origin=list(map(float,data['origin']))
    if not math.isfinite(resolution) or resolution<=0 or len(origin)!=3 or not all(map(math.isfinite,origin)):raise ValueError('Invalid metadata')
    compressed=base64.b64decode(data['cells'],validate=True)
    decoder=zlib.decompressobj();raw=decoder.decompress(compressed,w*h+1)
    if len(raw)!=w*h or not decoder.eof or decoder.unused_data or any(v>101 for v in raw):raise ValueError('Invalid payload')
    return w,h,resolution,origin,[v-1 for v in raw]
