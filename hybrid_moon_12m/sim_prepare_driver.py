"""Simulation preparation driver: motors always stopped, raw lidar preserved."""
import copy,math
from multirobot_sim.driver import MultiLimoDriver
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class Driver(MultiLimoDriver):
    def init(self,webots_node,properties):
        super().init(webots_node,properties)
        self.raw_pub=self.node.create_publisher(LaserScan,'scan_mapping',qos_profile_sensor_data)
        self.raw_due=0.
    def velocity(self,msg):
        self.command=(0.,0.)
    def step(self):
        self.enforce_preparation()
        super().step()
        now=self.robot.getTime()
        if now<self.raw_due:return
        self.raw_due=now+.5
        raw=self._LimoSensors__lidar.getRangeImage()[::-1]
        msg=copy.deepcopy(self._LimoSensors__scan)
        if len(raw)<2:return
        msg.ranges=[float(v) for v in raw]
        msg.angle_increment=(msg.angle_max-msg.angle_min)/(len(raw)-1)
        self.raw_pub.publish(msg)

    def enforce_preparation(self):
        self.stopped=True;self.command=(0.,0.)

class MissionDriver(Driver):
    def enforce_preparation(self):
        pass
    def velocity(self,msg):
        MultiLimoDriver.velocity(self,msg)
