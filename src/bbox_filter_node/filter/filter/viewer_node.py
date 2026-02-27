#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

names = {
    0: 'blue_armor',
    1: 'grey_armor',
    2: 'purple_armor',
    3: 'red_armor',
}


class BboxWithDistance(Node):
    QUEUE_SIZE = 10
    color = (0, 255, 0)
    bbox_thickness = 2

    def __init__(self):
        super().__init__('bbox_viewer')

        self.declare_parameter('flipped', False)
        self.flipped = self.get_parameter('flipped').get_parameter_value().bool_value

        self._bridge = cv_bridge.CvBridge()
        self._processed_image_pub = self.create_publisher(
            Image, '/yolov8_processed_image', self.QUEUE_SIZE)

        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/detections_output/with_pose')
        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            '/image')

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE)

        self.time_synchronizer.registerCallback(self.detections_callback)

    def detections_callback(self, detections_msg, img_msg):
        txt_color = (255, 0, 255)
        cv2_img = self._bridge.imgmsg_to_cv2(img_msg)

        if self.flipped:
            cv2_img = cv2.flip(cv2_img, -1)

        img_height = cv2_img.shape[0]
        img_width = cv2_img.shape[1]

        for detection in detections_msg.detections:
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y

            if self.flipped:
                center_x = img_width - center_x
                center_y = img_height - center_y

            if not detection.results:
                continue
            class_id = int(detection.results[0].hypothesis.class_id)
            label = names.get(class_id, 'unknown')
            pos = detection.results[0].pose.pose.position
            coordinates = f'x:{pos.x:.2f} y:{pos.y:.2f} z:{pos.z:.2f}'

            min_pt = (round(center_x - (width / 2.0)),
                      round(center_y - (height / 2.0)))
            max_pt = (round(center_x + (width / 2.0)),
                      round(center_y + (height / 2.0)))

            cv2.rectangle(cv2_img, min_pt, max_pt,
                          self.color, self.bbox_thickness)
            cv2.putText(cv2_img, coordinates, max_pt,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_color, 1)

        processed_img = self._bridge.cv2_to_imgmsg(
            cv2_img, encoding=img_msg.encoding)
        self._processed_image_pub.publish(processed_img)


def main():
    rclpy.init()
    rclpy.spin(BboxWithDistance())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
