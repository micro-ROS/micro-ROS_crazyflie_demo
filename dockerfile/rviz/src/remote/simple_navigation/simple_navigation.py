# Copyright (c) 2019 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/micro-ROS_kobuki_demo.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSReliabilityPolicy

class AttitudeToVel(Node):

    def __init__(self):
        super().__init__('attitude_to_vel')

        self.targetpoint = Vector3()
        self.targettheta = 0
        self.currenttwist = Twist()
        self.currentpoint = Vector3()

        self.sub_drone_att = self.create_subscription(Vector3, "/robot_pose", self.received_odometry, QoSProfile(depth=10))
        self.sub_drone_att = self.create_subscription(PoseStamped, "/move_base_simple/goal", self.clicked_point, QoSProfile(depth=10))
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=10))

        self.tf_timer = self.create_timer(0.1, self.send_vel)

    
    def clicked_point(self, rcv):
        self.targetpoint = rcv.pose.position
        self.targettheta = self.quaternion_to_euler(rcv.pose.orientation)[2]
        # print(rcv.point)

    def quaternion_to_euler(self,q):
        sqw = q.w * q.w
        sqx = q.x * q.x
        sqy = q.y * q.y
        sqz = q.z * q.z

        normal = math.sqrt(sqw + sqx + sqy + sqz)
        pole_result = (q.x * q.z) + (q.y * q.w)

        if (pole_result > (0.5 * normal)): # singularity at north pole
            ry = math.pi/2 #heading/yaw?
            rz = 0 #attitude/roll?
            rx = 2 * math.atan2(q.x, q.w) #bank/pitch?
            return Euler(rx, ry, rz)
        if (pole_result < (-0.5 * normal)): # singularity at south pole
            ry = -math.pi/2
            rz = 0
            rx = -2 * math.atan2(q.x, q.w)
            return Euler(rx, ry, rz)

        r11 = 2*(q.x*q.y + q.w*q.z)
        r12 = sqw + sqx - sqy - sqz
        r21 = -2*(q.x*q.z - q.w*q.y)
        r31 = 2*(q.y*q.z + q.w*q.x)
        r32 = sqw - sqx - sqy + sqz

        rx = math.atan2( r31, r32 )
        ry = math.asin ( r21 )
        rz = math.atan2( r11, r12 )

        return [rx, ry, rz]


    def angle_between(self,p1, p2):
        return math.atan2(p1.y-p2.y,p1.x-p2.x)

    def calculate_angle(self,p1, p2):
        ang1 = self.angle_between(p1,p2)
        ang2 = self.angle_between(p2,p1)
        print("{:0.2f} {:0.2f}".format(ang1,ang2))
        return ang1 if ang1 < ang2 else -ang2
    
    def received_odometry(self,rcv):
        self.currentpoint = rcv

    def send_vel(self):
        inc_x = self.targetpoint.x - self.currentpoint.x
        inc_y = self.targetpoint.y - self.currentpoint.y

        dist = math.sqrt(inc_x**2 + inc_y**2)

        theta = (self.currentpoint.z + math.pi) % (2* math.pi) 
  
        angle_to_goal = math.atan2(inc_y,inc_x) + math.pi - theta
        angle_to_target = self.targettheta + math.pi - theta    

        thr_pos = 0.3
        thr_angle = 0.2

        if dist > thr_pos and abs(angle_to_goal) > thr_angle:
            self.currenttwist.linear.x = 0.0
            self.currenttwist.angular.z = angle_to_goal * 0.3 + math.copysign(0.1,angle_to_goal)
        elif dist > thr_pos:
            self.currenttwist.linear.x = 0.1
            self.currenttwist.angular.z = 0.0
        elif dist < thr_pos and abs(angle_to_target) > thr_angle :
            self.currenttwist.linear.x = 0.0
            self.currenttwist.angular.z = angle_to_target * 0.3 + math.copysign(0.1,angle_to_target)

        self.pub_vel.publish(self.currenttwist)

        # print("Angle: {:0.2f}, Goal: {:0.2f}".format(theta,angle_to_goal))
        print("Target: {:0.2f},{:0.2f}|{:0.2f}\t Current {:0.2f},{:0.2f}|{:0.2f}\t Dist {:0.2f} Angle: {:0.2f} {:0.2f}".format(self.targetpoint.x,self.targetpoint.y,self.targettheta,self.currentpoint.x,self.currentpoint.y,self.currentpoint.z,dist,angle_to_goal,angle_to_target))


def main(args=None):
    rclpy.init(args=args)

    node = AttitudeToVel()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()