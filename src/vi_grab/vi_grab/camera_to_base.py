#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import ObjectInfo # pyright: ignore[reportMissingImports]


#手眼标定结果
ROT_CAM_TO_EE = np.array([
    [ 0.99984546, -0.00464906, -0.01695408],    # 0.01206237,  0.99929647,  0.03551135
    [ 0.00448498,  0.99994286, -0.00970348],    # -0.99988374,  0.01172294,  0.00975125
    [ 0.01699823,  0.00962594,  0.99980918]     # 0.00932809, -0.03562485,  0.9993217
])

TRANS_CAM_TO_EE = np.array([
     0.01213023,    # -0.08039019
    -0.06167299,    #  0.03225555
     0.026354     # -0.08256825
])


class CameraToBaseNode(Node):

    def __init__(self):
        super().__init__('camera_to_base_node')

        self.latest_arm_pose: Pose | None = None

        #订阅末端位姿（Pose）
        self.create_subscription(
            Pose,
            '/rm_driver/udp_arm_position',
            self.arm_pose_callback,
            10
        )

        #相机识别结果
        self.create_subscription(
            ObjectInfo,
            '/object_info',
            self.object_callback,
            10
        )

        #发布base坐标系物体
        self.pub = self.create_publisher(
            ObjectInfo,
            '/object_info_base',
            10
        )

        self.get_logger().info('Camera → Base transform node started.')


    def arm_pose_callback(self, msg: Pose):
        self.latest_arm_pose = msg


    def object_callback(self, msg: ObjectInfo):

        if self.latest_arm_pose is None:
            self.get_logger().warn('Waiting for arm pose...')
            return

        #相机坐标
        p_cam = np.array([msg.x, msg.y, msg.z])

        #T_camera_to_ee
        T_cam_ee = np.eye(4)
        T_cam_ee[:3, :3] = ROT_CAM_TO_EE
        T_cam_ee[:3, 3] = TRANS_CAM_TO_EE

        #T_base_to_ee
        pos = np.array([
            self.latest_arm_pose.position.x,
            self.latest_arm_pose.position.y,
            self.latest_arm_pose.position.z
        ])

        quat = [
            self.latest_arm_pose.orientation.x,
            self.latest_arm_pose.orientation.y,
            self.latest_arm_pose.orientation.z,
            self.latest_arm_pose.orientation.w
        ]

        rot = R.from_quat(quat).as_matrix()

        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = rot
        T_base_ee[:3, 3] = pos

        #坐标变换
        p_cam_h = np.append(p_cam, 1.0)
        p_ee_h  = T_cam_ee @ p_cam_h
        p_base  = (T_base_ee @ p_ee_h)[:3]

        #发布
        out = ObjectInfo()
        out.object_class = msg.object_class
        out.x = float(p_base[0])
        out.y = float(p_base[1])
        out.z = float(p_base[2])

        self.pub.publish(out)

        self.get_logger().info(
            f'{out.object_class} @ base: '
            f'[{out.x:.3f}, {out.y:.3f}, {out.z:.3f}]'
        )


def main():
    rclpy.init()
    node = CameraToBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
