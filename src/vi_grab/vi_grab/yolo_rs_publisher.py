#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2

from ultralytics import YOLO

from rm_ros_interfaces.msg import ObjectInfo # pyright: ignore[reportMissingImports]


class YoloRealSensePublisher(Node):

    def __init__(self):
        super().__init__('yolo_rs_publisher')

        #ROS2 参数  
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.declare_parameter('target_class', 'bottle')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value

        if model_path == '':
            self.get_logger().fatal('model_path 参数未设置')
            raise RuntimeError('model_path is required')

        #YOLO
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model: {model_path}')

        #RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

        self.intrinsics = (
            self.profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        #ROS2发布
        self.pub = self.create_publisher(ObjectInfo, '/object_info', 10)

        #定时器（30Hz）
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

        self.get_logger().info('YOLO RealSense node started.')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # YOLO 推理
        results = self.model(color_image, conf=0.5, verbose=False)[0]

        published = False

        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                conf = float(box.conf[0])

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                depth = depth_frame.get_distance(cx, cy)

                #可视化
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)

                label = f'{class_name} {conf:.2f}'
                if depth > 0:
                    label += f' {depth:.2f}m'

                cv2.putText(
                    color_image,
                    label,
                    (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

                #ROS发布，只发一个目标
                if published:
                    continue

                if self.target_class and class_name != self.target_class:
                    continue

                if depth <= 0.0:
                    continue

                point = rs.rs2_deproject_pixel_to_point(
                    self.intrinsics,
                    [cx, cy],
                    depth
                )

                msg = ObjectInfo()
                msg.object_class = class_name
                msg.x = float(point[0])
                msg.y = float(point[1])
                msg.z = float(point[2])

                self.pub.publish(msg)

                self.get_logger().info(
                    f'Object: {class_name} | '
                    f'camera xyz = ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
                )

                published = True

        #显示窗口
        cv2.imshow('YOLOv8 RealSense', color_image)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloRealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
