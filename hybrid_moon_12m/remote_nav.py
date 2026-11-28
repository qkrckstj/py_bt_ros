"""Laptop-side authenticated physical navigation transport; no auto-arm."""
import socket,time,uuid
from command_wire import pack,Reader,MAX_FRAME

class Link:
    def __init__(self,ip,port,key):
        self.address=(ip,port);self.key=key;self.sock=None;self.rx=b'';self.tx=b'';self.reader=None
        self.sequence=0;self.session=uuid.uuid4().hex;self.status={};self.received=-1e9;self.retry=0.
    def close(self):
        if self.sock:self.sock.close()
        self.sock=None;self.rx=b'';self.tx=b'';self.status={};self.received=-1e9
    def tick(self,payload):
        now=time.monotonic()
        if self.sock is None:
            if now<self.retry:return
            self.retry=now+1.
            try:
                self.sock=socket.create_connection(self.address,timeout=.1);self.sock.setblocking(False);self.reader=Reader(self.key)
            except OSError:self.close();return
        try:
            if not self.tx:
                self.sequence+=1;self.tx=pack(payload,self.key,self.sequence,self.session)
            try:
                n=self.sock.send(self.tx);self.tx=self.tx[n:]
            except BlockingIOError:pass
            try:
                block=self.sock.recv(MAX_FRAME)
                if not block:self.close();return
                self.rx+=block
            except BlockingIOError:pass
            if len(self.rx)>MAX_FRAME:raise ValueError('Overlong frame')
            while b'\n' in self.rx:
                raw,self.rx=self.rx.split(b'\n',1);self.status=self.reader.read(raw);self.received=now
        except (OSError,ValueError,KeyError,TypeError):self.close()
    def fresh(self):return time.monotonic()-self.received<.6

class RemoteNavigation:
    def __init__(self,link,route_provider):
        self.link,self.route_provider=link,route_provider;self.payload={'kind':'stop'};self.state='IDLE';self.id=None;self.local_ready=True;self.last_error=''
    def ready(self):return self.local_ready and self.link.fresh() and self.link.status.get('ready') is True
    def start(self,goal):
        try:reference,local,route=self.route_provider(goal)
        except (ValueError,KeyError,TypeError) as exc:
            self.last_error=str(exc);self.state='FAILED';self.payload={'kind':'stop'};return
        self.id=uuid.uuid4().hex
        self.payload={'kind':'goal','reference':reference,'task_id':self.id,'goal':list(local),'path':[list(p) for p in route]}
        self.state='SENDING'
    def cancel(self):
        self.payload={'kind':'stop'}
        if self.state in ('SENDING','NAVIGATING'):
            self.link.request_stop()
            self.state='CANCELING'
    def poll(self):
        if not self.link.fresh():
            if self.state in ('SENDING','NAVIGATING','CANCELING'):self.state='FAILED';self.payload={'kind':'stop'}
            return self.state
        if self.state=='CANCELING':
            if self.link.status.get('state') not in ('SENDING','NAVIGATING','CANCELING'):self.state='CANCELED'
        elif self.link.status.get('task_id')==self.id and self.id is not None:
            self.state=self.link.status.get('state','FAILED')
        return self.state

class LinkService:
    """Keep physical command leases alive independently of map/solver callbacks."""
    def __init__(self,ip,port,key):
        import threading
        self._lock=threading.RLock();self._stop=threading.Event()
        self._link=Link(ip,port,key);self._payload={'kind':'stop'};self._submitted=-1e9;self._deadline=float('inf')
        self._thread=threading.Thread(target=self._run,daemon=True);self._thread.start()
    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                now=time.monotonic()
                payload=self._payload if now-self._submitted<=.5 and now<self._deadline else {'kind':'stop'}
                self._link.tick(payload)
            self._stop.wait(.05)
    @property
    def status(self):
        with self._lock:return dict(self._link.status)
    def fresh(self):
        with self._lock:return self._link.fresh()
    def tick(self,payload):
        # No socket I/O in the ROS executor or CP-SAT callback.
        with self._lock:self._payload=payload;self._submitted=time.monotonic()
    def close(self):
        with self._lock:
            self._payload={'kind':'stop'};self._link.close()
    def request_stop(self):
        """Replace the outgoing goal without interrupting status reception."""
        with self._lock:
            self._payload = {'kind': 'stop'}
            self._submitted = time.monotonic()
            # A partially written frame cannot be replaced on this stream.
            if self._link.tx:
                self._link.close()

    def set_deadline(self,deadline):
        with self._lock:self._deadline=deadline
    def shutdown(self):
        self._stop.set();self._thread.join(timeout=.3)
        self.close()
