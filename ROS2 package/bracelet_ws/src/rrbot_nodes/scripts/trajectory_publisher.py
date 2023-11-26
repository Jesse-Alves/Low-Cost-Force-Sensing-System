#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

m1_ = 1.0
l1_ = 1.0
lc1_ = l1_/2
i1_ = m1_/12*l1_*l1_
m2_ = 1.0
l2_ = 1.0
lc2_ = l2_/2
i2_ = m2_/12*l2_*l2_
g_ = -9.8

def rrbot_direct_kinematics(q1, q2):
    px = l1_*math.cos(q1)+(l1_+l2_)*math.cos(q1+q2)
    py = l1_*math.sin(q1)+(l1_+l2_)*math.sin(q1+q2)
    return px, py

def rrbot_inverse_kinematics(px, py):
    a = (px*px+py*py-l1_*l1_-l2_*l2_)/(2*l1_*l2_)
    b = -math.sqrt(1-a*a)
    q2 = math.atan2(b,a)

    c = (px*(l1_+l2_*math.cos(q2))+py*l2_*math.sin(q2))/(px*px + py*py)
    d = math.sqrt(1-c*c)    
    q1 = math.atan2(d,c)

    return q1, q2


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/rrbot_idyn_controller/commands', 1)

        # Get the bracelet data
        self.contraction = 0.0
        self.minimal = (5.0)*math.pi/180.0
        self.maximum = (150.0)*math.pi/180.0
        self.q1 = self.maximum
        self.q2 = self.minimal
        self.bracelet_subscription = self.create_subscription(Float64,'/fsr_topic', self.bracelet_callback, 1)
        self.bracelet_subscription 

        # Publisher the angles
        
        #self.timer_period = 0.002  # seconds
        #self.timer = self.create_timer(self.timer_period, self.timer_callback)
        #self.t = 0

    def bracelet_callback(self, msg_contraction):

        # Get data via subscription
        self.contraction = msg_contraction.data


        # Mimic the Robot
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1','joint2']
        point = JointTrajectoryPoint()

        #px = 0.5*math.cos(0.5*self.t) + 1.0
        #py = 0.5*math.sin(0.5*self.t) 
        #q1, q2 = rrbot_inverse_kinematics(px, py)

        # Keep fixed
        self.q1 = -(85.0)*math.pi/180.0
        self.q2 = self.maximum*(self.contraction/100.0)
        
        #self.get_logger().info(f"q2 = {self.q2}")
        #self.get_logger().info(f"contraction = {self.contraction}")

        point.positions = [self.q1, self.q2]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()