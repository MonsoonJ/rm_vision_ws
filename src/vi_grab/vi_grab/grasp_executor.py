#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import ( # pyright: ignore[reportMissingImports]
    Movejp,
    Movel,
    Gripperpick,
    ObjectInfo
)


class GraspExecutorPick(Node):
    def __init__(self):
        super().__init__('grasp_executor_pick')

        #订阅
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

        #发布
        self.movejp_pub = self.create_publisher(
            Movejp,
            '/rm_driver/movej_p_cmd',
            10
        )

        self.movel_pub = self.create_publisher(
            Movel,
            '/rm_driver/movel_cmd',
            10
        )

        self.gripper_pub = self.create_publisher(
            Gripperpick,
            '/rm_driver/set_gripper_pick_cmd',
            10
        )

        self.current_pose: Pose | None = None
        self.target_obj: ObjectInfo | None = None
        self.executed = False

        self.get_logger().info('Grasp executor with pick & retreat started.')

    #回调

    def arm_pose_callback(self, msg: Pose):
        self.current_pose = msg

    def object_callback(self, msg: ObjectInfo):
        if self.executed or self.current_pose is None:
            return

        self.executed = True
        self.target_obj = msg

        self.pre_grasp(msg)

    #运动阶段

    # def pre_grasp(self, obj: ObjectInfo):
    #     approach_dx = 0.12
    #     approach_dz = 0.10
    #     min_z = 0.35

    #     cmd = Movejp()
    #     cmd.pose.position.x = obj.x - approach_dx
    #     cmd.pose.position.y = obj.y
    #     cmd.pose.position.z = max(obj.z + approach_dz, min_z)
    #     cmd.pose.orientation = self.current_pose.orientation
    #     cmd.speed = 30

    #     self.movejp_pub.publish(cmd)
    #     self.get_logger().info('Pre-grasp pose sent.')

    #     self.create_timer(2.0, self.linear_approach)

    def linear_approach(self):
        obj = self.target_obj

        cmd = Movel()
        cmd.pose.position.x = obj.x
        cmd.pose.position.y = obj.y
        cmd.pose.position.z = obj.z
        cmd.pose.orientation = self.current_pose.orientation
        cmd.speed = 30

        self.movel_pub.publish(cmd)
        self.get_logger().info('Linear approach sent.')

        self.create_timer(1.5, self.close_gripper)

    def close_gripper(self):
        cmd = Gripperpick()
        cmd.speed = 200
        cmd.force = 800

        self.gripper_pub.publish(cmd)
        self.get_logger().info('Gripper close command sent.')

        self.create_timer(1.5, self.retreat)

    def retreat(self):
        obj = self.target_obj

        cmd = Movel()
        cmd.pose.position.x = obj.x
        cmd.pose.position.y = obj.y
        cmd.pose.position.z = obj.z + 0.10   #垂直上抬
        cmd.pose.orientation = self.current_pose.orientation
        cmd.speed = 30

        self.movel_pub.publish(cmd)
        self.get_logger().info('Retreat motion sent. Grasp cycle complete.')


def main():
    rclpy.init()
    node = GraspExecutorPick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
