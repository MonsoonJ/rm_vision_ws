#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import (
    Movejp,
    Movel,
    Gripperpick,
    Gripperset,
    ObjectInfo
)


class GraspExecutor(Node):

    def __init__(self):
        super().__init__('grasp_executor_with_retract')

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
        self.target = None
        self.executed = False
        self.state = 'IDLE'  #定义状态枚举
        # self.ox = 0.0
        # self.oy = 0.0
        # self.oz = 1.0

        self.get_logger().info('Grasp executor with retract (fixed) started.')


    def arm_pose_callback(self, msg: Pose):
        self.current_pose = msg

    def object_callback(self, msg: ObjectInfo):
        if self.executed or self.current_pose is None:
            return

        self.executed = True
        self.target = msg

        self.get_logger().info(
            f'Received target [{msg.object_class}] @ '
            f'({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
        )

        self.move_above_target()

    # MoveJ_P 到目标前方
    def move_above_target(self):
        if self.state != 'IDLE':
            return
        self.state = 'MOVE_ABOVE'
        cmd = Movejp()
        cmd.pose.position.x = self.target.x + 0.2  # 提前一定距离
        cmd.pose.position.y = self.target.y + 0.0
        cmd.pose.position.z = self.target.z + 0.0
        cmd.pose.orientation = self.current_pose.orientation
        cmd.speed = 30 #速度%

        self.movejp_pub.publish(cmd)
        self.get_logger().info('MoveJ_P above target.')

        self.create_timer(1.0, self.open_gripper)

    # 打开夹爪
    def open_gripper(self):
        if self.state != 'MOVE_ABOVE':
            return
        self.state = 'OPEN_GRIPPER'
        cmd = Gripperset()
        cmd.position = 1000
        self.gripper_set_pub.publish(cmd)

        self.get_logger().info('Gripper opened.')
        self.create_timer(4.0, self.move_down)

    # MoveL 前探抓取
    def move_down(self):
        if self.state != 'OPEN_GRIPPER':
            return
        self.state = 'MOVE_DOWN'
        cmd = Movel()
        cmd.pose.position.x = self.target.x + 0.08  #爪子的长度
        cmd.pose.position.y = self.target.y + 0.0
        cmd.pose.position.z = self.target.z + 0.0
        cmd.pose.orientation = self.current_pose.orientation
        cmd.speed = 30

        self.movel_pub.publish(cmd)
        self.get_logger().info('MoveL down to target.')

        self.create_timer(2.0, self.close_gripper)

    #  闭合夹爪
    def close_gripper(self):
        if self.state != 'MOVE_DOWN':
            return
        self.state = 'CLOSE_GRIPPER'
        cmd = Gripperpick()
        cmd.speed = 200
        cmd.force = 500
        self.gripper_pick_pub.publish(cmd)

        self.get_logger().info('Gripper closed.')
        self.create_timer(2.0, self.retract)

    #  回撤
    def retract(self):
        if self.state != 'CLOSE_GRIPPER':
            return
        self.state = 'DONE'
        cmd = Movel()
        cmd.pose.position.x = self.target.x + 0.5  #回撤距离
        cmd.pose.position.y = self.target.y + 0.4
        cmd.pose.position.z = self.target.z + 0.15 
        cmd.pose.orientation = self.current_pose.orientation
        cmd.speed = 30

        self.movel_pub.publish(cmd)
        self.get_logger().info('Retract completed.')
    #     self.create_timer(3.0, self.open_gripper_final)
    
    #     # 打开夹爪
    # def open_gripper_final(self):
    #     if self.state != 'MOVE_ABOVE':
    #         return
    #     self.state = 'OPEN_GRIPPER'
    #     cmd = Gripperset()
    #     cmd.position = 1000
    #     self.gripper_set_pub.publish(cmd)

    #     self.get_logger().info('Gripper opened.')


def main():
    rclpy.init()
    node = GraspExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
