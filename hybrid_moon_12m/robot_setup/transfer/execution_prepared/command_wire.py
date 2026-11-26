"""Authenticated, bounded command envelopes; clocks must be synchronized."""
import hashlib,hmac,json,math,time
MAX_FRAME=32768

def pack(payload,key,sequence,session):
    body=json.dumps({'sent':time.time(),'sequence':sequence,'session':session,'payload':payload},sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    out=json.dumps({'body':body.decode(),'mac':hmac.new(key,body,hashlib.sha256).hexdigest()},separators=(',',':')).encode()+b'\n'
    if len(out)>MAX_FRAME:raise ValueError('Frame too large')
    return out

class Reader:
    def __init__(self,key):self.key=key;self.session=None;self.sequence=-1;self.retired=set()
    def read(self,raw,now=None):
        if len(raw)>MAX_FRAME:raise ValueError('Frame too large')
        envelope=json.loads(raw);body=envelope['body'].encode()
        if not hmac.compare_digest(envelope['mac'],hmac.new(self.key,body,hashlib.sha256).hexdigest()):raise ValueError('Authentication failed')
        value=json.loads(body);stamp=value['sent'];seq=value['sequence'];session=value['session']
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not -.25<=(time.time() if now is None else now)-stamp<=1.:raise ValueError('Stale packet or clock skew')
        if type(seq)!=int or seq<0 or not isinstance(session,str) or len(session)>100 or session in self.retired:raise ValueError('Invalid session')
        if self.session==session and seq<=self.sequence:raise ValueError('Replayed packet')
        if self.session and self.session!=session:self.retired.add(self.session)
        self.session,self.sequence=session,seq
        return value['payload']
