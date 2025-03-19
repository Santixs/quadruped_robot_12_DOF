#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.do_publish()

    def do_publish(self):
        if self.i == 0:
            joint_state_msg = JointState()
            joint_state_msg.name = [
                'Front_Right_Foot_Point', 'Front_Left_Foot_Point',
                'Back_Right_Foot_Point', 'Back_Left_Foot_Point', 'Yaw_rotation'
            ]

            # Fixed test positions
            joint_state_msg.x = [0.1, -0.1, 0.2, -0.2, 0.0]
            joint_state_msg.y = [0.05, -0.05, 0.1, -0.1, 0.0]
            joint_state_msg.z = [0.2, 0.2, 0.15, 0.15, 0.0]

            self.get_logger().info(f'Publishing: {joint_state_msg}')
            self.publisher_.publish(joint_state_msg)

        self.i += 1

    def timer_callback(self):
        quit()


def main(args=None):
    rclpy.init(args=args)
    joint_state_pub = JointStatePublisher()

    while rclpy.ok():
        rclpy.spin_once(joint_state_pub)

    #joint_state_pub.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
