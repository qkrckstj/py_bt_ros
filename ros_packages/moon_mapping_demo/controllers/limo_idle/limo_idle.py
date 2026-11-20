"""Hold four lunar-demo LIMOs stationary; no autonomous or ROS control."""
from controller import Robot
robot = Robot()
step = int(robot.getBasicTimeStep())
for name in ('front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'):
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
while robot.step(step) != -1:
    pass
