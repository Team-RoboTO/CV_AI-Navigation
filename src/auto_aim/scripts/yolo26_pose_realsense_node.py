#!/usr/bin/env python3
"""TensorRT YOLOv26-pose detector for RealSense color images.

Publishes:
  /detector/armors           vision_msgs/Detection2DArray
  /detector/armors_keypoints auto_aim/ArmorKeypointArray
  /yolo/debug_image          sensor_msgs/Image, optional raw image copy

The keypoint order is fixed for the C++ PnP path: TL, TR, BR, BL.
"""

from typing import List, Optional, Tuple

import cv2
import cupy as cp
import numpy as np
import rclpy
import tensorrt as trt
from auto_aim.msg import ArmorKeypoint, ArmorKeypointArray
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


DEFAULT_ENGINE_PATH = "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine"
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_FRAME_ID = "camera_color_optical_frame"

NUM_KEYPOINTS = 4
KEYPOINT_NAMES = ["TL", "TR", "BR", "BL"]
DEFAULT_CLASS_NAMES = ["blue_armor", "grey_armor", "red_armor", "red_armor_alt"]

MAX_CANDIDATES = 80
MAX_DETECTIONS = 30
PAD_VALUE = 0.447


class Yolo26PoseRealSenseNode(Node):
    def __init__(self):
        super().__init__("yolo26_pose_realsense_node")

        self.declare_parameter("engine_path", DEFAULT_ENGINE_PATH)
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("threshold", 0.20)
        self.declare_parameter("nms_iou", 0.20)
        self.declare_parameter("keypoint_score_threshold", 0.05)
        self.declare_parameter("publish_debug_every", 0)
        self.declare_parameter("debug_scores", True)
        self.declare_parameter("class_names", DEFAULT_CLASS_NAMES)
        self.declare_parameter("device_id", 0)

        self.engine_path = str(self.get_parameter("engine_path").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.threshold = float(self.get_parameter("threshold").value)
        self.nms_iou = float(self.get_parameter("nms_iou").value)
        self.keypoint_score_threshold = float(
            self.get_parameter("keypoint_score_threshold").value
        )
        self.publish_debug_every = int(self.get_parameter("publish_debug_every").value)
        self.debug_scores = bool(self.get_parameter("debug_scores").value)
        self.class_names = list(self.get_parameter("class_names").value)
        self.device_id = int(self.get_parameter("device_id").value)

        self.device = cp.cuda.Device(self.device_id)
        self.device.use()
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.trt_logger, "")

        with open(self.engine_path, "rb") as f:
            runtime = trt.Runtime(self.trt_logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"Could not deserialize TensorRT engine: {self.engine_path}")

        self.trt_ctx = self.engine.create_execution_context()
        self.in_name = self.engine.get_tensor_name(0)
        self.out_name = self.engine.get_tensor_name(1)

        in_shape = tuple(int(x) for x in self.engine.get_tensor_shape(self.in_name))
        out_shape = tuple(int(x) for x in self.engine.get_tensor_shape(self.out_name))
        if any(x <= 0 for x in in_shape + out_shape):
            raise RuntimeError(
                f"Dynamic TensorRT shape detected: input={in_shape} output={out_shape}. "
                "Export with dynamic=False for this real-time node."
            )

        self.img_size, self.input_channels = self._parse_input_shape(in_shape)
        if self.input_channels != 3:
            raise RuntimeError(f"Expected 3-channel NCHW input, got {in_shape}")

        self.output_mode, self.output_channels, self.num_rows, self.channels_first = (
            self._parse_output_shape(out_shape)
        )
        if self.output_mode == "post_nms":
            self.num_classes = len(self.class_names)
        else:
            self.num_classes = self.output_channels - 4 - NUM_KEYPOINTS * 3
            if self.num_classes <= 0:
                raise RuntimeError(
                    f"Invalid raw pose output channels: {self.output_channels}"
                )

        self.h_out = np.empty(int(np.prod(out_shape)), dtype=np.float32)
        self.d_in = cp.empty(
            self.input_channels * self.img_size * self.img_size,
            dtype=cp.float32,
        )
        self.d_out = cp.empty_like(cp.asarray(self.h_out))
        self.stream = cp.cuda.Stream(non_blocking=True)

        self.native_w: Optional[int] = None
        self.native_h: Optional[int] = None
        self.d_bgra = None
        self.pre_fn = None
        self.scale_native_to_net = 1.0
        self.scale_net_to_native = 1.0
        self.content_w_net = self.img_size
        self.content_h_net = self.img_size
        self.pad_x_net = 0
        self.pad_y_net = 0

        self.boxes_buf = np.empty((MAX_CANDIDATES, 4), dtype=np.float32)
        self.kpts_buf = np.empty((MAX_CANDIDATES, NUM_KEYPOINTS, 3), dtype=np.float32)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.det_pub = self.create_publisher(Detection2DArray, "/detector/armors", qos)
        self.kpt_pub = self.create_publisher(
            ArmorKeypointArray, "/detector/armors_keypoints", qos
        )
        self.debug_pub = self.create_publisher(Image, "/yolo/debug_image", 1)
        self.bridge = CvBridge()
        self.frame_count = 0

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        self.get_logger().info(
            f"YOLOv26 RealSense detector ready: engine={self.engine_path}, "
            f"input={in_shape}, output={out_shape}, mode={self.output_mode}, "
            f"layout={'[C,N]' if self.channels_first else '[N,C]'}, "
            f"image_topic={self.image_topic}"
        )

    def _parse_input_shape(self, in_shape: Tuple[int, ...]) -> Tuple[int, int]:
        dims = [int(d) for d in in_shape]
        if len(dims) == 4 and dims[1] in (1, 3) and dims[2] == dims[3]:
            return int(dims[2]), int(dims[1])
        raise RuntimeError(f"Unsupported input shape {in_shape}; expected static NCHW square")

    def _parse_output_shape(self, out_shape: Tuple[int, ...]) -> Tuple[str, int, int, bool]:
        dims = [int(d) for d in out_shape if int(d) != 1]
        if len(dims) != 2:
            raise RuntimeError(f"Unsupported output shape {out_shape}")

        a, b = dims
        post_nms_c = 6 + NUM_KEYPOINTS * 3
        if b == post_nms_c:
            return "post_nms", b, a, False
        if a == post_nms_c:
            return "post_nms", a, b, True
        if a <= 256 and b > a:
            return "raw", a, b, True
        if b <= 256 and a > b:
            return "raw", b, a, False
        raise RuntimeError(f"Could not infer YOLO-pose output layout from {out_shape}")

    def _ensure_image_geometry(self, width: int, height: int):
        if self.native_w == width and self.native_h == height and self.pre_fn is not None:
            return

        self.native_w = int(width)
        self.native_h = int(height)
        self.scale_native_to_net = min(
            float(self.img_size) / float(width),
            float(self.img_size) / float(height),
        )
        self.scale_net_to_native = 1.0 / self.scale_native_to_net
        self.content_w_net = int(round(width * self.scale_native_to_net))
        self.content_h_net = int(round(height * self.scale_native_to_net))
        self.pad_x_net = int((self.img_size - self.content_w_net) // 2)
        self.pad_y_net = int((self.img_size - self.content_h_net) // 2)

        self.d_bgra = cp.empty((height, width, 4), dtype=cp.uint8)
        self.pre_fn = self._compile_preprocess_kernel()
        self.get_logger().info(
            f"Preprocess geometry: {width}x{height} -> {self.img_size}x{self.img_size}, "
            f"content={self.content_w_net}x{self.content_h_net}, "
            f"pad=({self.pad_x_net},{self.pad_y_net}), "
            f"scale={self.scale_native_to_net:.6f}"
        )

    def _compile_preprocess_kernel(self):
        source = f"""
        extern "C" __global__ void pre(const unsigned char* in, float* out,
                                       int in_w, int in_h, int out_w, int out_h) {{
            int x = blockIdx.x * blockDim.x + threadIdx.x;
            int y = blockIdx.y * blockDim.y + threadIdx.y;
            if (x >= out_w || y >= out_h) return;

            int o = y * out_w + x;
            int pad_x = {self.pad_x_net};
            int pad_y = {self.pad_y_net};
            int content_w = {self.content_w_net};
            int content_h = {self.content_h_net};
            int stride = {self.img_size * self.img_size};

            if (x < pad_x || x >= pad_x + content_w ||
                y < pad_y || y >= pad_y + content_h) {{
                out[o] = {PAD_VALUE}f;
                out[o + stride] = {PAD_VALUE}f;
                out[o + 2 * stride] = {PAD_VALUE}f;
                return;
            }}

            int sx = (int)((x - pad_x) * {self.scale_net_to_native:.8f}f);
            int sy = (int)((y - pad_y) * {self.scale_net_to_native:.8f}f);
            if (sx < 0) sx = 0;
            if (sy < 0) sy = 0;
            if (sx >= in_w) sx = in_w - 1;
            if (sy >= in_h) sy = in_h - 1;
            int i = (sy * in_w + sx) * 4;

            out[o] = (float)in[i + 2] / 255.0f;
            out[o + stride] = (float)in[i + 1] / 255.0f;
            out[o + 2 * stride] = (float)in[i + 0] / 255.0f;
        }}
        """
        return cp.RawKernel(source, "pre")

    def _output_as_rows(self) -> np.ndarray:
        if self.channels_first:
            return self.h_out.reshape(self.output_channels, self.num_rows).T
        return self.h_out.reshape(self.num_rows, self.output_channels)

    def _net_x_to_native(self, x_net: np.ndarray) -> np.ndarray:
        return (x_net - self.pad_x_net) * self.scale_net_to_native

    def _net_y_to_native(self, y_net: np.ndarray) -> np.ndarray:
        return (y_net - self.pad_y_net) * self.scale_net_to_native

    def _decode_raw_pose_candidates(self, output: np.ndarray, valid_idx: np.ndarray):
        v_data = output[valid_idx]
        class_scores = v_data[:, 4:4 + self.num_classes]
        v_confs = np.max(class_scores, axis=1)
        v_classes = np.argmax(class_scores, axis=1).astype(np.int32)

        n = len(v_data)
        boxes = self.boxes_buf[:n]
        xywh = v_data[:, :4].astype(np.float32, copy=True)
        finite_xywh = np.abs(xywh[np.isfinite(xywh)])
        if finite_xywh.size and np.max(finite_xywh) <= 2.0:
            xywh *= float(self.img_size)

        boxes[:, 2] = xywh[:, 2] * self.scale_net_to_native
        boxes[:, 3] = xywh[:, 3] * self.scale_net_to_native
        boxes[:, 0] = self._net_x_to_native(xywh[:, 0]) - boxes[:, 2] * 0.5
        boxes[:, 1] = self._net_y_to_native(xywh[:, 1]) - boxes[:, 3] * 0.5

        kpt_start = 4 + self.num_classes
        kpt_raw = v_data[:, kpt_start:kpt_start + NUM_KEYPOINTS * 3]
        kpts = self.kpts_buf[:n]
        kpts[:] = kpt_raw.reshape(n, NUM_KEYPOINTS, 3)
        kpt_abs = np.abs(kpts[:, :, :2])
        kpt_finite = kpt_abs[np.isfinite(kpt_abs)]
        if kpt_finite.size and np.max(kpt_finite) <= 2.0:
            kpts[:, :, 0] *= float(self.img_size)
            kpts[:, :, 1] *= float(self.img_size)
        kpts[:, :, 0] = self._net_x_to_native(kpts[:, :, 0])
        kpts[:, :, 1] = self._net_y_to_native(kpts[:, :, 1])
        return boxes, kpts, v_confs, v_classes

    def _decode_post_nms_rows(self, output: np.ndarray):
        confs = output[:, 4]
        valid_idx = np.nonzero(np.isfinite(confs) & (confs > self.threshold))[0]
        if len(valid_idx) == 0:
            return None, None, None, None

        if len(valid_idx) > MAX_DETECTIONS:
            top_local = np.argpartition(confs[valid_idx], -MAX_DETECTIONS)[-MAX_DETECTIONS:]
            valid_idx = valid_idx[top_local]

        v_data = output[valid_idx]
        n = len(v_data)
        boxes = self.boxes_buf[:n]

        xyxy = v_data[:, :4].astype(np.float32, copy=True)
        finite_xyxy = np.abs(xyxy[np.isfinite(xyxy)])
        if finite_xyxy.size and np.max(finite_xyxy) <= 2.0:
            xyxy *= float(self.img_size)

        x1n = self._net_x_to_native(xyxy[:, 0])
        y1n = self._net_y_to_native(xyxy[:, 1])
        x2n = self._net_x_to_native(xyxy[:, 2])
        y2n = self._net_y_to_native(xyxy[:, 3])

        boxes[:, 0] = np.minimum(x1n, x2n)
        boxes[:, 1] = np.minimum(y1n, y2n)
        boxes[:, 2] = np.abs(x2n - x1n)
        boxes[:, 3] = np.abs(y2n - y1n)

        kpt_raw = v_data[:, 6:6 + NUM_KEYPOINTS * 3]
        kpts = self.kpts_buf[:n]
        kpts[:] = kpt_raw.reshape(n, NUM_KEYPOINTS, 3)
        kpt_abs = np.abs(kpts[:, :, :2])
        kpt_finite = kpt_abs[np.isfinite(kpt_abs)]
        if kpt_finite.size and np.max(kpt_finite) <= 2.0:
            kpts[:, :, 0] *= float(self.img_size)
            kpts[:, :, 1] *= float(self.img_size)
        kpts[:, :, 0] = self._net_x_to_native(kpts[:, :, 0])
        kpts[:, :, 1] = self._net_y_to_native(kpts[:, :, 1])

        return boxes, kpts, v_data[:, 4].astype(np.float32), np.rint(v_data[:, 5]).astype(np.int32)

    def image_callback(self, msg: Image):
        try:
            bgra = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")
            bgra = np.ascontiguousarray(bgra)
            height, width = bgra.shape[:2]

            self.device.use()
            with self.stream:
                self._ensure_image_geometry(width, height)
                self.d_bgra.set(bgra, stream=self.stream)
                self.pre_fn(
                    ((self.img_size + 15) // 16, (self.img_size + 15) // 16),
                    (16, 16, 1),
                    (
                        self.d_bgra,
                        self.d_in,
                        np.int32(width),
                        np.int32(height),
                        np.int32(self.img_size),
                        np.int32(self.img_size),
                    ),
                    stream=self.stream,
                )
                self.trt_ctx.set_tensor_address(self.in_name, int(self.d_in.data.ptr))
                self.trt_ctx.set_tensor_address(self.out_name, int(self.d_out.data.ptr))
                self.trt_ctx.execute_async_v3(self.stream.ptr)
                self.d_out.get(out=self.h_out, stream=self.stream)
                self.stream.synchronize()

            output = self._output_as_rows()
            boxes = kpts = v_confs = v_classes = None
            indices: List[int] = []

            if self.output_mode == "post_nms":
                if self.debug_scores and self.frame_count % 60 == 0 and output.size:
                    finite_conf = output[:, 4][np.isfinite(output[:, 4])]
                    max_conf = float(np.max(finite_conf)) if finite_conf.size else 0.0
                    self.get_logger().info(
                        f"post_nms max_conf={max_conf:.3f} threshold={self.threshold:.3f}"
                    )
                decoded = self._decode_post_nms_rows(output)
                if decoded[0] is not None:
                    boxes, kpts, v_confs, v_classes = decoded
                    indices = list(range(len(v_confs)))
            else:
                class_scores = output[:, 4:4 + self.num_classes]
                confs = np.max(class_scores, axis=1)
                if self.debug_scores and self.frame_count % 60 == 0:
                    max_conf = float(np.nanmax(confs)) if confs.size else 0.0
                    self.get_logger().info(
                        f"raw max_conf={max_conf:.3f} threshold={self.threshold:.3f}"
                    )
                valid_idx = np.nonzero(confs > self.threshold)[0]
                if len(valid_idx) > 0:
                    if len(valid_idx) > MAX_CANDIDATES:
                        top_local = np.argpartition(confs[valid_idx], -MAX_CANDIDATES)[-MAX_CANDIDATES:]
                        valid_idx = valid_idx[top_local]
                    boxes, kpts, v_confs, v_classes = self._decode_raw_pose_candidates(output, valid_idx)
                    indices = cv2.dnn.NMSBoxes(
                        boxes.tolist(), v_confs.tolist(), self.threshold, self.nms_iou
                    )

            self._publish_detections(msg, width, height, boxes, kpts, v_confs, v_classes, indices)

            self.frame_count += 1
            if self.publish_debug_every > 0 and self.frame_count % self.publish_debug_every == 0:
                dbg = self.bridge.cv2_to_imgmsg(bgra, encoding="bgra8")
                dbg.header = msg.header
                self.debug_pub.publish(dbg)
        except Exception as exc:
            self.get_logger().error(f"YOLOv26 RealSense callback failed: {exc}", throttle_duration_sec=2.0)

    def _publish_detections(
        self,
        image_msg: Image,
        width: int,
        height: int,
        boxes,
        kpts,
        v_confs,
        v_classes,
        indices,
    ):
        det_array = Detection2DArray()
        det_array.header = image_msg.header
        det_array.header.frame_id = self.frame_id or image_msg.header.frame_id

        kpt_array = ArmorKeypointArray()
        kpt_array.header = det_array.header
        kpt_array.image_width = int(width)
        kpt_array.image_height = int(height)
        kpt_array.keypoint_order = KEYPOINT_NAMES

        if boxes is not None and len(indices) > 0:
            for raw_i in np.array(indices).flatten()[:MAX_DETECTIONS]:
                left_raw, top_raw, box_w_raw, box_h_raw = boxes[int(raw_i)]
                left = max(0.0, min(float(width), float(left_raw)))
                top = max(0.0, min(float(height), float(top_raw)))
                right = max(0.0, min(float(width), float(left_raw + box_w_raw)))
                bottom = max(0.0, min(float(height), float(top_raw + box_h_raw)))
                box_w = right - left
                box_h = bottom - top
                if box_w <= 1.0 or box_h <= 1.0:
                    continue

                cls = int(v_classes[int(raw_i)])
                conf = float(v_confs[int(raw_i)])
                kp = kpts[int(raw_i)].copy()
                kp[:, 0] = np.clip(kp[:, 0], 0.0, float(width))
                kp[:, 1] = np.clip(kp[:, 1], 0.0, float(height))

                det = Detection2D()
                det.header = det_array.header
                det.bbox.center.position.x = float(left + box_w * 0.5)
                det.bbox.center.position.y = float(top + box_h * 0.5)
                det.bbox.size_x = float(box_w)
                det.bbox.size_y = float(box_h)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls)
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

                kmsg = ArmorKeypoint()
                kmsg.header = det_array.header
                kmsg.class_id = cls
                kmsg.class_name = self.class_names[cls] if 0 <= cls < len(self.class_names) else str(cls)
                kmsg.confidence = conf
                kmsg.bbox_cx = float(left + box_w * 0.5)
                kmsg.bbox_cy = float(top + box_h * 0.5)
                kmsg.bbox_w = float(box_w)
                kmsg.bbox_h = float(box_h)
                kmsg.keypoints_xy = [
                    float(kp[0, 0]), float(kp[0, 1]),
                    float(kp[1, 0]), float(kp[1, 1]),
                    float(kp[2, 0]), float(kp[2, 1]),
                    float(kp[3, 0]), float(kp[3, 1]),
                ]
                kmsg.keypoint_scores = [float(kp[j, 2]) for j in range(NUM_KEYPOINTS)]
                kmsg.keypoint_valid = [
                    bool(kp[j, 2] >= self.keypoint_score_threshold)
                    for j in range(NUM_KEYPOINTS)
                ]
                kpt_array.detections.append(kmsg)

        self.det_pub.publish(det_array)
        self.kpt_pub.publish(kpt_array)

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = Yolo26PoseRealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
