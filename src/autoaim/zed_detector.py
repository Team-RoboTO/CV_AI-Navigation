#!/usr/bin/env python3

"""
ZED X Mini + TensorRT YOLO26-pose detector for RoboMaster armors.

This version is adapted from the bbox-only detector. It still publishes the
legacy /detector/armors Detection2DArray topic, but it also publishes a JSON
message containing bbox + the 4 keypoints in this convention:

    KP0 = TL, KP1 = TR, KP2 = BR, KP3 = BL

Topics:
    /detector/armors                vision_msgs/Detection2DArray
    /detector/armors_keypoints      autoaim/ArmorKeypointArray
    /detector/armors_keypoints_json std_msgs/String
    /yolo/debug_image               sensor_msgs/Image
    /zed/imu_data                   sensor_msgs/Imu
    /camera_info                    sensor_msgs/CameraInfo

Supported TensorRT output layouts:
    1) Raw YOLO pose output, exported with nms=False:
       [1, C, A] or [1, A, C]
       C = 4 + NUM_CLASSES + NUM_KEYPOINTS * 3
         = bbox(cx,cy,w,h) + class scores + [x,y,score] per keypoint

    2) Post-NMS YOLO pose output, exported with nms=True or end2end:
       [1, max_det, 6 + NUM_KEYPOINTS * 3]
       row = x1,y1,x2,y2,conf,class,kp0_x,kp0_y,kp0_score,...

The node auto-detects which one your engine uses.

Preprocessing auto-adapts to the TensorRT engine input size:
    native 960x600 BGRA -> NET_SIZExNET_SIZE RGB NCHW
    example NET_SIZE=640: resized content=640x400, pad_y=120
    example NET_SIZE=960: resized content=960x600, pad_y=180

Inverse mapping back to ZED native pixels uses the same dynamic letterbox geometry.
"""

import json
import math
import os
import threading
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
import pycuda.driver as cuda
import pyzed.sl as sl
import rclpy
import tensorrt as trt
from cv_bridge import CvBridge
from pycuda.compiler import SourceModule
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from autoaim.msg import ArmorKeypoint, ArmorKeypointArray
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

# -----------------------------------------------------------------------------
# CONFIG
# -----------------------------------------------------------------------------
def find_models_dir(start: Path) -> Path:
    for path in (start, *start.parents):
        candidate = path / "models"
        if candidate.is_dir():
            return candidate
    return start / "models"


def default_engine_path() -> str:
    env_path = os.environ.get("AUTOAIM_ENGINE_PATH")
    if env_path:
        return env_path
    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory("autoaim"))
            return str(find_models_dir(share_dir) / "jetson64" / "yolov26_keypoints.engine")
        except Exception:
            pass
    return str(find_models_dir(Path(__file__).resolve().parent) / "jetson64" / "yolov26_keypoints.engine")


ENGINE_PATH = default_engine_path()

# ZED X Mini SVGA @ 120 fps: left image is 960x600.
NATIVE_W, NATIVE_H = 960, 600
IMG_SIZE = 640
GRAB_FPS = 120

# Model / dataset convention.
NUM_KEYPOINTS = 4
KEYPOINT_NAMES = ["TL", "TR", "BR", "BL"]
CLASS_NAMES = {
    0: "blue_armor",
    1: "grey_armor",
    2: "red_armor",
}

THRESHOLD = 0.15
NMS_IOU = 0.2
MAX_CANDIDATES = 80
MAX_DETECTIONS = 30
KEYPOINT_SCORE_THRESHOLD = 0.05

FRAME_ID = "camera_color_optical_frame"
IMU_FRAME_ID = "zed_imu_link"

# Preprocess geometry is computed from the engine input shape at runtime.


class Yolo26PoseKeypointsNode(Node):
    def __init__(self):
        super().__init__("yolo26_pose_keypoints_node")

        # Parameters let you override constants from launch/CLI without editing.
        self.declare_parameter("engine_path", ENGINE_PATH)
        self.declare_parameter("threshold", THRESHOLD)
        self.declare_parameter("nms_iou", NMS_IOU)
        self.declare_parameter("publish_debug_every", 4)
        self.declare_parameter("debug_scores", True)

        self.engine_path = self.get_parameter("engine_path").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.nms_iou = float(self.get_parameter("nms_iou").value)
        self.publish_debug_every = int(self.get_parameter("publish_debug_every").value)
        self.debug_scores = bool(self.get_parameter("debug_scores").value)

        # ── 1. CUDA & TensorRT ───────────────────────────────────────────────
        cuda.init()
        self.cuda_ctx = cuda.Device(0).make_context()
        self.logger_trt = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger_trt, "")

        with open(self.engine_path, "rb") as f:
            self.engine = trt.Runtime(self.logger_trt).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"Could not deserialize TensorRT engine: {self.engine_path}")

        self.trt_ctx = self.engine.create_execution_context()
        self.in_name = self.engine.get_tensor_name(0)
        self.out_name = self.engine.get_tensor_name(1)

        out_shape = tuple(int(x) for x in self.engine.get_tensor_shape(self.out_name))
        if any(x <= 0 for x in out_shape):
            # Static export is strongly recommended. This branch only catches mistakes early.
            raise RuntimeError(
                f"Dynamic output shape {out_shape} detected. Export with dynamic=False for this node."
            )

        self.out_shape = out_shape

        in_shape = tuple(int(x) for x in self.engine.get_tensor_shape(self.in_name))
        self.get_logger().info(f"Engine input shape={in_shape}")
        self.img_size, self.input_channels = self._parse_input_shape(in_shape)
        self.channel_stride = self.img_size * self.img_size

        # Letterbox geometry for native 960x600 -> square engine input.
        self.scale_net_to_native = float(NATIVE_W) / float(self.img_size)
        self.unscaled_h_net = int(round(float(NATIVE_H) / self.scale_net_to_native))
        self.pad_y_net = int((self.img_size - self.unscaled_h_net) // 2)

        # self.get_logger().info(
        #     f"Preprocess: {NATIVE_W}x{NATIVE_H} -> {self.img_size}x{self.img_size}, "
        #     f"content={self.img_size}x{self.unscaled_h_net}, pad_y={self.pad_y_net}, "
        #     f"net_to_native_scale={self.scale_net_to_native:.6f}"
        # )

        self.output_mode, self.output_channels, self.num_anchors, self.channels_first = self._parse_output_shape(out_shape)
        if self.output_mode == "post_nms":
            # Post-NMS format is [x1,y1,x2,y2,conf,class,kpts...].
            # Number of classes is not encoded in the output width.
            self.num_classes = len(CLASS_NAMES)
        else:
            self.num_classes = self.output_channels - 4 - NUM_KEYPOINTS * 3
            if self.num_classes <= 0:
                raise RuntimeError(
                    f"Invalid raw pose output channels: C={self.output_channels}. Expected "
                    f"4 + classes + {NUM_KEYPOINTS}*3. Engine output shape: {out_shape}"
                )

        # self.get_logger().info(
        #     f"Engine: {self.engine_path}\n"
        #     f"Output shape={out_shape}, mode={self.output_mode}, channels={self.output_channels}, "
        #     f"rows/anchors={self.num_anchors}, classes={self.num_classes}, "
        #     f"layout={'[C,A]' if self.channels_first else '[A,C]'}"
        # )

        # ── 2. GPU buffers ──────────────────────────────────────────────────
        self.h_out = cuda.pagelocked_empty(int(np.prod(out_shape)), dtype=np.float32)
        self.d_in = cuda.mem_alloc(self.input_channels * self.img_size * self.img_size * 4)
        self.d_out = cuda.mem_alloc(self.h_out.nbytes)
        self.d_rgba = cuda.mem_alloc(NATIVE_W * NATIVE_H * 4)
        self.stream = cuda.Stream()

        self.boxes_buf = np.empty((MAX_CANDIDATES, 4), dtype=np.float32)
        self.kpts_buf = np.empty((MAX_CANDIDATES, NUM_KEYPOINTS, 3), dtype=np.float32)

        # ── 3. CUDA preprocessing kernel ────────────────────────────────────
        # Dynamic for engine input size, e.g. 960x600 -> 960x960 or 640x640.
        mod = SourceModule(f"""
        __global__ void pre(const unsigned char* in, float* out,
                            int in_w, int in_h, int out_w, int out_h) {{
            int x = blockIdx.x * blockDim.x + threadIdx.x;
            int y = blockIdx.y * blockDim.y + threadIdx.y;
            if (x < out_w && y < out_h) {{
                int pad_y      = {self.pad_y_net};
                int unscaled_h = {self.unscaled_h_net};
                int o = y * out_w + x;
                if (y < pad_y || y >= (pad_y + unscaled_h)) {{
                    out[o]                              = 0.447f;
                    out[o + {self.channel_stride}]      = 0.447f;
                    out[o + {2 * self.channel_stride}]  = 0.447f;
                }} else {{
                    int sx = (int)(x * {self.scale_net_to_native:.8f}f);
                    int sy = (int)((y - pad_y) * {self.scale_net_to_native:.8f}f);
                    if (sx >= in_w) sx = in_w - 1;
                    if (sy >= in_h) sy = in_h - 1;
                    int i = (sy * in_w + sx) * 4;
                    out[o]                              = (float)in[i+2] / 255.0f;  // R
                    out[o + {self.channel_stride}]      = (float)in[i+1] / 255.0f;  // G
                    out[o + {2 * self.channel_stride}]  = (float)in[i]   / 255.0f;  // B
                }}
            }}
        }}
        """)
        self.pre_fn = mod.get_function("pre")
        self.cuda_ctx.pop()

        # ── 4. ROS publishers ───────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Detection2DArray, "/detector/armors", qos)
        # Typed keypoint output consumed by autoaim.
        self.kpt_pub = self.create_publisher(ArmorKeypointArray, "/detector/armors_keypoints", qos)
        # Optional JSON debug copy; useful for quick echo/debug without custom message introspection.
        self.kpt_json_pub = self.create_publisher(String, "/detector/armors_keypoints_json", qos)
        self.zed_pub = self.create_publisher(Imu, "/zed/imu_data", qos)
        self.img_pub = self.create_publisher(Image, "/yolo/debug_image", 1)
        self.cam_info_pub = self.create_publisher(CameraInfo, "/camera_info", qos)
        self.bridge = CvBridge()
        self.frame_count = 0

        # ── 5. ZED Camera ───────────────────────────────────────────────────
        self.zed = sl.Camera()
        params = sl.InitParameters()
        params.camera_resolution = sl.RESOLUTION.SVGA
        params.camera_fps = GRAB_FPS
        params.depth_mode = sl.DEPTH_MODE.NONE
        params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        # Uncomment if camera is physically mounted upside-down:
        params.camera_image_flip = sl.FLIP_MODE.ON

        err = self.zed.open(params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Could not open ZED camera: {err}")

        # Exposure settings for dark arenas. Tune on the real field.
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 30)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 20)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)

        # Camera intrinsics for /camera_info.
        calib = (
            self.zed.get_camera_information()
            .camera_configuration
            .calibration_parameters
            .left_cam
        )
        self.cam_matrix = np.array(
            [[calib.fx, 0, calib.cx], [0, calib.fy, calib.cy], [0, 0, 1]],
            dtype=np.float64,
        )

        self.initial_orientation = None
        self.mat_cpu = sl.Mat()
        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()
        #self.get_logger().info("YOLO26 pose keypoint detector started")

    # ------------------------------------------------------------------
    # TensorRT input parsing helpers
    # ------------------------------------------------------------------
    def _parse_input_shape(self, in_shape: Tuple[int, ...]) -> Tuple[int, int]:
        """Return (img_size, channels) from static NCHW TensorRT input shape."""
        dims = [int(d) for d in in_shape]
        if len(dims) != 4 or any(d <= 0 for d in dims):
            raise RuntimeError(f"Unsupported/dynamic input shape {in_shape}. Export with dynamic=False.")

        # Common Ultralytics TensorRT: [1, 3, H, W].
        if dims[1] in (1, 3) and dims[2] == dims[3]:
            return int(dims[2]), int(dims[1])

        # Less common NHWC: [1, H, W, 3]. This node writes NCHW.
        if dims[3] in (1, 3) and dims[1] == dims[2]:
            raise RuntimeError(
                f"NHWC input shape {in_shape} detected. This node writes NCHW. "
                f"Export the model as standard Ultralytics NCHW TensorRT."
            )

        raise RuntimeError(f"Could not infer square NCHW input from shape {in_shape}.")

    # ------------------------------------------------------------------
    # TensorRT output parsing helpers
    # ------------------------------------------------------------------
    def _parse_output_shape(self, out_shape: Tuple[int, ...]) -> Tuple[str, int, int, bool]:
        """Return (mode, channels, rows_or_anchors, channels_first).

        raw mode:
            [1, C, A] or [1, A, C]
            C = 4 + classes + nkpt*3

        post_nms mode:
            [1, max_det, 6 + nkpt*3]
            row = x1,y1,x2,y2,conf,class,kpts...
        """
        dims = [d for d in out_shape if d != 1]
        if len(dims) != 2:
            raise RuntimeError(f"Unsupported output shape: {out_shape}")

        a, b = dims
        post_nms_c = 6 + NUM_KEYPOINTS * 3

        # Your current engine logs [1, 300, 18]. For 4 KPs, 18 = 6 + 12,
        # so this is a post-NMS/end2end export, not raw YOLO anchors.
        if b == post_nms_c:
            return "post_nms", b, a, False
        if a == post_nms_c:
            return "post_nms", a, b, True

        # Raw YOLO pose: one dimension is small C, the other is many anchors.
        if a <= 256 and b > a:
            return "raw", a, b, True
        if b <= 256 and a > b:
            return "raw", b, a, False

        raise RuntimeError(f"Could not infer output layout from shape: {out_shape}")

    def _raw_output_as_rows(self) -> np.ndarray:
        """Return output as [rows, channels]."""
        if self.channels_first:
            return self.h_out.reshape(self.output_channels, self.num_anchors).T
        return self.h_out.reshape(self.num_anchors, self.output_channels)

    def _net_x_to_native(self, x_net: np.ndarray) -> np.ndarray:
        return x_net * self.scale_net_to_native

    def _net_y_to_native(self, y_net: np.ndarray) -> np.ndarray:
        return (y_net - self.pad_y_net) * self.scale_net_to_native

    def _decode_raw_pose_candidates(self, output: np.ndarray, valid_idx: np.ndarray):
        """Decode raw YOLO pose rows in native 960x600 pixels."""
        v_data = output[valid_idx]
        class_scores = v_data[:, 4:4 + self.num_classes]
        v_confs = np.max(class_scores, axis=1)
        v_classes = np.argmax(class_scores, axis=1).astype(np.int32)

        n = len(v_data)
        boxes = self.boxes_buf[:n]

        # bbox cx,cy,w,h in network 640x640 letterboxed space -> native LTWH.
        boxes[:, 2] = v_data[:, 2] * self.scale_net_to_native
        boxes[:, 3] = v_data[:, 3] * self.scale_net_to_native
        boxes[:, 0] = v_data[:, 0] * self.scale_net_to_native - boxes[:, 2] * 0.5
        boxes[:, 1] = (v_data[:, 1] - self.pad_y_net) * self.scale_net_to_native - boxes[:, 3] * 0.5

        # keypoints x,y,score per kp.
        kpt_start = 4 + self.num_classes
        kpt_raw = v_data[:, kpt_start:kpt_start + NUM_KEYPOINTS * 3]
        kpts = self.kpts_buf[:n]
        kpts[:] = kpt_raw.reshape(n, NUM_KEYPOINTS, 3)
        kpts[:, :, 0] = self._net_x_to_native(kpts[:, :, 0])
        kpts[:, :, 1] = self._net_y_to_native(kpts[:, :, 1])

        return boxes, kpts, v_confs, v_classes

    def _decode_post_nms_rows(self, output: np.ndarray):
        """Decode post-NMS/end2end rows in native 960x600 pixels.

        Expected row:
            x1,y1,x2,y2,conf,class,kp0_x,kp0_y,kp0_score,...

        Coordinates are assumed to be in the 640x640 letterboxed model input.
        They are converted back to native ZED left image coordinates 960x600.
        """
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

        x1 = v_data[:, 0]
        y1 = v_data[:, 1]
        x2 = v_data[:, 2]
        y2 = v_data[:, 3]

        # If the exporter returns normalized coordinates, convert to pixels first.
        # Most TensorRT nms=True exports return 640-space pixels, but this makes
        # the parser robust.
        coord_abs = np.abs(v_data[:, :4])
        coord_finite = coord_abs[np.isfinite(coord_abs)]
        if coord_finite.size and np.max(coord_finite) <= 2.0:
            x1 = x1 * self.img_size
            x2 = x2 * self.img_size
            y1 = y1 * self.img_size
            y2 = y2 * self.img_size

        x1n = self._net_x_to_native(x1)
        x2n = self._net_x_to_native(x2)
        y1n = self._net_y_to_native(y1)
        y2n = self._net_y_to_native(y2)

        boxes[:, 0] = np.minimum(x1n, x2n)
        boxes[:, 1] = np.minimum(y1n, y2n)
        boxes[:, 2] = np.abs(x2n - x1n)
        boxes[:, 3] = np.abs(y2n - y1n)

        kpt_raw = v_data[:, 6:6 + NUM_KEYPOINTS * 3]
        kpts = self.kpts_buf[:n]
        kpts[:] = kpt_raw.reshape(n, NUM_KEYPOINTS, 3)

        # Robust handling for normalized keypoints.
        kpt_abs = np.abs(kpts[:, :, :2])
        kpt_finite = kpt_abs[np.isfinite(kpt_abs)]
        if kpt_finite.size and np.max(kpt_finite) <= 2.0:
            kpts[:, :, 0] *= self.img_size
            kpts[:, :, 1] *= self.img_size

        kpts[:, :, 0] = self._net_x_to_native(kpts[:, :, 0])
        kpts[:, :, 1] = self._net_y_to_native(kpts[:, :, 1])

        v_confs = v_data[:, 4].astype(np.float32)
        v_classes = v_data[:, 5].astype(np.int32)
        return boxes, kpts, v_confs, v_classes

    def _make_keypoint_json(self, now_msg, detections: List[Dict]) -> str:
        payload = {
            "header": {
                "stamp": {"sec": int(now_msg.sec), "nanosec": int(now_msg.nanosec)},
                "frame_id": FRAME_ID,
            },
            "image_width": NATIVE_W,
            "image_height": NATIVE_H,
            "keypoint_order": KEYPOINT_NAMES,
            "detections": detections,
        }
        return json.dumps(payload, separators=(",", ":"))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def loop(self):
        while self.running:
            if self.zed.grab(sl.RuntimeParameters()) != sl.ERROR_CODE.SUCCESS:
                continue

            now_msg = self.get_clock().now().to_msg()
            self.zed.retrieve_image(self.mat_cpu, sl.VIEW.LEFT, sl.MEM.CPU)

            # TensorRT inference.
            self.cuda_ctx.push()
            cuda.memcpy_htod_async(self.d_rgba, self.mat_cpu.get_data(), self.stream)
            self.pre_fn(
                self.d_rgba,
                self.d_in,
                np.int32(NATIVE_W),
                np.int32(NATIVE_H),
                np.int32(self.img_size),
                np.int32(self.img_size),
                block=(16, 16, 1),
                grid=((self.img_size + 15) // 16, (self.img_size + 15) // 16),
                stream=self.stream,
            )
            self.trt_ctx.set_tensor_address(self.in_name, int(self.d_in))
            self.trt_ctx.set_tensor_address(self.out_name, int(self.d_out))
            self.trt_ctx.execute_async_v3(self.stream.handle)
            cuda.memcpy_dtoh_async(self.h_out, self.d_out, self.stream)
            self.stream.synchronize()
            self.cuda_ctx.pop()

            output = self._raw_output_as_rows()

            det_array_msg = Detection2DArray()
            det_array_msg.header.stamp = now_msg
            det_array_msg.header.frame_id = FRAME_ID

            kpt_array_msg = ArmorKeypointArray()
            kpt_array_msg.header.stamp = now_msg
            kpt_array_msg.header.frame_id = FRAME_ID
            kpt_array_msg.image_width = NATIVE_W
            kpt_array_msg.image_height = NATIVE_H
            kpt_array_msg.keypoint_order = KEYPOINT_NAMES

            json_detections: List[Dict] = []

            if self.output_mode == "post_nms":
                finite_conf = output[:, 4][np.isfinite(output[:, 4])] if output.size else np.array([], dtype=np.float32)
                max_conf = float(np.max(finite_conf)) if finite_conf.size else 0.0
                # if self.debug_scores and self.frame_count % 60 == 0:
                    # self.get_logger().info(f"post_nms max_conf={max_conf:.3f} threshold={self.threshold:.3f}")
                decoded = self._decode_post_nms_rows(output)
                if decoded[0] is None:
                    indices = []
                    boxes = kpts = v_confs = v_classes = None
                else:
                    boxes, kpts, v_confs, v_classes = decoded
                    # Engine already applied NMS, so do not run cv2 NMS again.
                    indices = np.arange(len(v_confs), dtype=np.int32)
            else:
                class_scores = output[:, 4:4 + self.num_classes]
                confs = np.max(class_scores, axis=1)
                max_conf = float(np.nanmax(confs)) if confs.size else 0.0
                # if self.debug_scores and self.frame_count % 60 == 0:
                    # self.get_logger().info(f"raw max_conf={max_conf:.3f} threshold={self.threshold:.3f}")
                valid_idx = np.nonzero(confs > self.threshold)[0]
                if len(valid_idx) == 0:
                    indices = []
                    boxes = kpts = v_confs = v_classes = None
                else:
                    # Keep top candidates before NMS for speed.
                    if len(valid_idx) > MAX_CANDIDATES:
                        top_local = np.argpartition(confs[valid_idx], -MAX_CANDIDATES)[-MAX_CANDIDATES:]
                        valid_idx = valid_idx[top_local]
                    boxes, kpts, v_confs, v_classes = self._decode_raw_pose_candidates(output, valid_idx)
                    indices = cv2.dnn.NMSBoxes(
                        boxes.tolist(), v_confs.tolist(), self.threshold, self.nms_iou
                    )

            if boxes is not None and len(indices) > 0:
                for raw_i in np.array(indices).flatten()[:MAX_DETECTIONS]:
                        L_r, T_r, W_r, H_r = boxes[raw_i]
                        L = max(0.0, min(float(NATIVE_W), float(L_r)))
                        T = max(0.0, min(float(NATIVE_H), float(T_r)))
                        R = max(0.0, min(float(NATIVE_W), float(L_r + W_r)))
                        B = max(0.0, min(float(NATIVE_H), float(T_r + H_r)))
                        W = R - L
                        H = B - T
                        if W <= 1.0 or H <= 1.0:
                            continue

                        cls = int(v_classes[raw_i])
                        conf = float(v_confs[raw_i])
                        kp = kpts[raw_i].copy()
                        kp[:, 0] = np.clip(kp[:, 0], 0.0, float(NATIVE_W))
                        kp[:, 1] = np.clip(kp[:, 1], 0.0, float(NATIVE_H))

                        # Legacy bbox topic for current autoaim compatibility.
                        det = Detection2D()
                        det.header.stamp = now_msg
                        det.header.frame_id = FRAME_ID
                        det.bbox.center.position.x = float(L + W * 0.5)
                        det.bbox.center.position.y = float(T + H * 0.5)
                        det.bbox.size_x = float(W)
                        det.bbox.size_y = float(H)

                        hyp = ObjectHypothesisWithPose()
                        hyp.hypothesis.class_id = str(cls)
                        hyp.hypothesis.score = conf
                        det.results.append(hyp)
                        det_array_msg.detections.append(det)

                        # New typed keypoint output for autoaim.
                        kmsg = ArmorKeypoint()
                        kmsg.header.stamp = now_msg
                        kmsg.header.frame_id = FRAME_ID
                        kmsg.class_id = int(cls)
                        kmsg.class_name = CLASS_NAMES.get(cls, str(cls))
                        kmsg.confidence = float(conf)
                        kmsg.bbox_cx = float(L + W * 0.5)
                        kmsg.bbox_cy = float(T + H * 0.5)
                        kmsg.bbox_w = float(W)
                        kmsg.bbox_h = float(H)
                        kmsg.keypoints_xy = [
                            float(kp[0, 0]), float(kp[0, 1]),  # TL
                            float(kp[1, 0]), float(kp[1, 1]),  # TR
                            float(kp[2, 0]), float(kp[2, 1]),  # BR
                            float(kp[3, 0]), float(kp[3, 1]),  # BL
                        ]
                        kmsg.keypoint_scores = [float(kp[j, 2]) for j in range(NUM_KEYPOINTS)]
                        kmsg.keypoint_valid = [
                            bool(kp[j, 2] >= KEYPOINT_SCORE_THRESHOLD)
                            for j in range(NUM_KEYPOINTS)
                        ]
                        kpt_array_msg.detections.append(kmsg)

                        # Optional JSON debug copy.
                        json_detections.append({
                            "class_id": cls,
                            "class_name": CLASS_NAMES.get(cls, str(cls)),
                            "confidence": conf,
                            "bbox": {
                                "left": float(L),
                                "top": float(T),
                                "right": float(R),
                                "bottom": float(B),
                                "cx": float(L + W * 0.5),
                                "cy": float(T + H * 0.5),
                                "w": float(W),
                                "h": float(H),
                            },
                            "keypoints": [
                                {
                                    "name": KEYPOINT_NAMES[j],
                                    "x": float(kp[j, 0]),
                                    "y": float(kp[j, 1]),
                                    "score": float(kp[j, 2]),
                                    "valid": bool(kp[j, 2] >= KEYPOINT_SCORE_THRESHOLD),
                                }
                                for j in range(NUM_KEYPOINTS)
                            ],
                        })

            self.pub.publish(det_array_msg)
            self.kpt_pub.publish(kpt_array_msg)
            self.kpt_json_pub.publish(String(data=self._make_keypoint_json(now_msg, json_detections)))

            # IMU and camera info.
            self._publish_imu(now_msg)
            self._publish_camera_info(now_msg)

            # Raw image for the viewer node. Publish less often to reduce bandwidth.
            self.frame_count += 1
            if self.publish_debug_every > 0 and self.frame_count % self.publish_debug_every == 0:
                img_msg = self.bridge.cv2_to_imgmsg(self.mat_cpu.get_data(), "bgra8")
                img_msg.header.stamp = now_msg
                img_msg.header.frame_id = FRAME_ID
                self.img_pub.publish(img_msg)

    # ------------------------------------------------------------------
    # IMU publisher: same logic as your bbox-only node.
    # ------------------------------------------------------------------
    def _publish_imu(self, now_msg):
        sensors_data = sl.SensorsData()
        # if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            # self.get_logger().warn("Failed to get sensors data", throttle_duration_sec=5.0)
            # return

        imu_data = sensors_data.get_imu_data()
        # if not imu_data.is_available:
            # self.get_logger().warn("IMU not available on this ZED model", throttle_duration_sec=5.0)
            # return

        accel = np.array(imu_data.get_linear_acceleration())
        gyro = np.array(imu_data.get_angular_velocity())
        gyro_rad = np.deg2rad(gyro)

        q_curr = np.array(imu_data.get_pose().get_orientation().get())
        norm = float(np.linalg.norm(q_curr))
        # if abs(norm - 1.0) > 0.1:
            # self.get_logger().warn(
                # f"IMU orientation not yet valid (norm={norm:.3f}), waiting...",
                # throttle_duration_sec=2.0,
            # )
            # return

        if self.initial_orientation is None:
            self.initial_orientation = q_curr.copy()
            # self.get_logger().info(
                # f"IMU reference captured: x={q_curr[0]:.4f} y={q_curr[1]:.4f} "
                # f"z={q_curr[2]:.4f} w={q_curr[3]:.4f}"
            #)

        qi = self.initial_orientation
        qci = np.array([-qi[0], -qi[1], -qi[2], qi[3]])

        w1, x1, y1, z1 = qci[3], qci[0], qci[1], qci[2]
        w2, x2, y2, z2 = q_curr[3], q_curr[0], q_curr[1], q_curr[2]

        dw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        dx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        dy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        dz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        nd = math.sqrt(dw * dw + dx * dx + dy * dy + dz * dz)
        if nd < 1e-6:
            dw, dx, dy, dz = 1.0, 0.0, 0.0, 0.0
        else:
            dw /= nd
            dx /= nd
            dy /= nd
            dz /= nd

        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME_ID
        imu_msg.header.stamp = now_msg
        imu_msg.orientation.w = float(dw)
        imu_msg.orientation.x = float(dx)
        imu_msg.orientation.y = float(dy)
        imu_msg.orientation.z = float(dz)
        imu_msg.orientation_covariance[0] = 0.001
        imu_msg.angular_velocity.x = float(gyro_rad[0])
        imu_msg.angular_velocity.y = float(gyro_rad[1])
        imu_msg.angular_velocity.z = float(gyro_rad[2])
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.zed_pub.publish(imu_msg)

    # ------------------------------------------------------------------
    # Camera info publisher
    # ------------------------------------------------------------------
    def _publish_camera_info(self, now_msg):
        calib = (
            self.zed.get_camera_information()
            .camera_configuration
            .calibration_parameters
            .left_cam
        )
        msg = CameraInfo()
        msg.header.stamp = now_msg
        msg.header.frame_id = FRAME_ID
        msg.width = NATIVE_W
        msg.height = NATIVE_H
        msg.k = [
            calib.fx, 0.0, calib.cx,
            0.0, calib.fy, calib.cy,
            0.0, 0.0, 1.0,
        ]
        msg.d = [0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"
        self.cam_info_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.cuda_ctx.push()
            self.cuda_ctx.pop()
        except Exception:
            pass
        self.zed.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Yolo26PoseKeypointsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
