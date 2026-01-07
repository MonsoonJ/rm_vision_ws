#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import (
    Movejp,
    Gripperpick,
    Gripperset,
    ObjectInfo
)


class GraspExecutorMinimal(Node):

    def __init__(self):
        super().__init__('grasp_executor_minimal')

        # -------- subscribers --------
        self.create_subscription(
            ObjectInfo,
            '/object_info_base',
            self.object_callback,
            10
        )

        self.create_subscription(
            Pose,
            '/rm_driver/udp_arm_position',
            self.arm_pose_callback,
            10
        )

        # -------- publishers --------
        self.movejp_pub = self.create_publisher(
            Movejp,
            '/rm_driver/movej_p_cmd',
            10
        )

        self.gripper_pick_pub = self.create_publisher(
            Gripperpick,
            '/rm_driver/set_gripper_pick_cmd',
            10
        )

        self.gripper_set_pub = self.create_publisher(
            Gripperset,
            '/rm_driver/set_gripper_position_cmd',
            10
        )

        self.current_pose = None
        self.executed = False

        self.get_logger().info('Minimal grasp executor started.')

    def arm_pose_callback(self, msg: Pose):
        self.current_pose = msg

    def object_callback(self, msg: ObjectInfo):
        if self.executed or self.current_pose is None:
            return

        self.executed = True
        self.get_logger().info(
            f'Received target [{msg.object_class}] @ '
            f'({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
        )

        self.move_to_target(msg)

    def move_to_target(self, obj: ObjectInfo):
        """仅 MoveJ_P 到目标正上方"""

        cmd = Movejp()

        cmd.pose.position.x = obj.x
        cmd.pose.position.y = obj.y
        cmd.pose.position.z = obj.z + 0.10   # 固定上方 10cm

        # 姿态：完全保持当前机械臂姿态
        cmd.pose.orientation = self.current_pose.orientation

        cmd.speed = 20

        self.movejp_pub.publish(cmd)
        self.get_logger().info('MoveJ_P to target sent.')

        # 延时执行夹爪
        self.create_timer(2.0, self.open_gripper)

    def open_gripper(self):
        cmd = Gripperset()
        cmd.position = 1000   # 完全打开

        self.gripper_set_pub.publish(cmd)
        self.get_logger().info('Gripper open.')

        self.create_timer(1.5, self.close_gripper)

    def close_gripper(self):
        cmd = Gripperpick()
        cmd.speed = 200
        cmd.force = 600

        self.gripper_pick_pub.publish(cmd)
        self.get_logger().info('Gripper close. Minimal sequence done.')


def main():
    rclpy.init()
    node = GraspExecutorMinimal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
