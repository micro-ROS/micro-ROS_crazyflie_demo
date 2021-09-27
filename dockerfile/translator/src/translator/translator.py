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

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSReliabilityPolicy

class AttitudeToVel(Node):

    def __init__(self):
        super().__init__('translator')

        self.posearray = []

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub_drone_tf = self.create_subscription(TransformStamped, "/drone/tf", self.drone_tf_callback, qos_profile)
        self.sub_drone_pose = self.create_subscription(PoseStamped, "/drone/pose", self.drone_pose_callback, qos_profile)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=1))
        self.pub_posearray = self.create_publisher(Path, "/drone/path", QoSProfile(depth=1))

    def drone_tf_callback(self, tf):
        # Wrap and publish TF
        tfmsg = TFMessage()
        tfmsg.transforms = [tf]
        self.pub_tf.publish(tfmsg)

    def drone_pose_callback(self, pose):
        # Publish Rviz Path
        msgpath = Path()
        msgpath.header.frame_id = "/map"
        msgpath.header.stamp = Clock().now().to_msg()

        self.posearray.append(pose)
        if len(self.posearray) > 50:
            self.posearray = self.posearray[1:]

        msgpath.poses = self.posearray
        self.pub_posearray.publish(msgpath)


def main(args=None):
    rclpy.init(args=args)

    node = AttitudeToVel()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
