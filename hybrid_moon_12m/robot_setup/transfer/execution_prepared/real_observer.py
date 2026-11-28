"""Run on the real robot. Subscribes to map/TF; never commands motion."""
import argparse,json,socket,time,uuid
from pathlib import Path
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
from map_wire import encode_map,MAX_PACKET

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--target',required=True);parser.add_argument('--port',type=int,default=17442);a=parser.parse_args()
    rclpy.init();n=rclpy.create_node('hybrid_readonly_observer');b=Buffer();l=TransformListener(b,n)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);cache=[None];seq=[0];session=uuid.uuid4().hex
    def receive(m):
        if m.header.frame_id!='map':return
        try:cache[0]=encode_map(m)
        except (ValueError,OverflowError) as e:n.get_logger().warning(str(e))
    sub=n.create_subscription(OccupancyGrid,'/map',receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    reference_path=Path(__file__).with_name('real_reference.json')
    reference=json.loads(reference_path.read_text()) if reference_path.is_file() else None
    def send():
        nonlocal reference
        if cache[0] is None:return
        pose=None
        try:
            t=b.lookup_transform('map','base_link',rclpy.time.Time())
            age=(n.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
            if -.25<=age<=1.:
                import math
                q=t.transform.rotation;p=t.transform.translation
                pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        except Exception:pass
        if reference is None and pose is not None:
            reference={'id':uuid.uuid4().hex,'anchor':pose}
            reference_path.write_text(json.dumps(reference,allow_nan=False)+'\n')
        seq[0]+=1
        packet=json.dumps({'version':1,'session':session,'sequence':seq[0],'map':cache[0],'pose':pose,'reference':reference},allow_nan=False).encode()
        if len(packet)>MAX_PACKET:
            n.get_logger().warning('Map exceeds datagram limit; not transmitted');return
        sock.sendto(packet,(a.target,a.port))
    n.create_timer(.5,send)
    n.get_logger().info('Read-only observer: no motion command publishers or action clients')
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:sock.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
