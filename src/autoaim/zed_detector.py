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
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
import pycuda.driver as cuda
import pyzed.sl as sl
import rclpy
import tensorrt as trt
from rclpy.time import Time as RclpyTime
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
# Default TensorRT engine path. The model is kept OUTSIDE the package (e.g.
# on the Isaac ROS dev volume), so it is not installed by colcon. Resolution
# order: ROS param engine_path  >  env AUTOAIM_ENGINE_PATH  >  this default.
DEFAULT_ENGINE = "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine"


def default_engine_path() -> str:
    return os.environ.get("AUTOAIM_ENGINE_PATH", DEFAULT_ENGINE)


ENGINE_PATH = default_engine_path()

# ZED resolution name -> (per-eye left-image width, height). The left-image
# size drives the letterbox geometry and the GPU buffers, so it must be known
# before the CUDA kernel is compiled. SVGA = 960x600 is the value this detector
# historically hard-coded. The sl.RESOLUTION enum is looked up by name.
ZED_RESOLUTIONS = {
    "VGA":    (672, 376),
    "SVGA":   (960, 600),
    "HD720":  (1280, 720),
    "HD1080": (1920, 1080),
    "HD1200": (1920, 1200),
}

# Model / dataset convention. These are TRUE invariants of the trained model
# (class id order and keypoint order) and must NOT become tunable parameters.
NUM_KEYPOINTS = 4
KEYPOINT_NAMES = ["TL", "TR", "BR", "BL"]
CLASS_NAMES = {
    0: "blue_armor",
    1: "grey_armor",
    2: "red_armor",
}

# Decode-stage array sizing. Algorithm invariants, not field-tuning values.
MAX_CANDIDATES = 80
MAX_DETECTIONS = 30
KEYPOINT_SCORE_THRESHOLD = 0.05

# Defaults below preserve the original hard-coded behavior. Every one of them is
# now a ROS parameter (see config/zed.yaml) so the camera/runtime can be
# tuned from launch/YAML without editing this file.
IMG_SIZE = 640  # fallback only; the real net size is read from the engine.
DEFAULT_RESOLUTION = "SVGA"
DEFAULT_FPS = 120
DEFAULT_IMAGE_FLIP = True
DEFAULT_AUTO_EXPOSURE = False
DEFAULT_EXPOSURE = 60
DEFAULT_GAIN = 50
DEFAULT_AUTO_WHITE_BALANCE = True
DEFAULT_THRESHOLD = 0.15
DEFAULT_NMS_IOU = 0.2
DEFAULT_PUBLISH_DEBUG_EVERY = 4
DEFAULT_CAMERA_INFO_EVERY = 1
DEFAULT_FRAME_ID = "camera_color_optical_frame"
DEFAULT_IMU_FRAME_ID = "zed_imu_link"

# Preprocess geometry is computed from the engine input shape at runtime.


class Yolo26PoseKeypointsNode(Node):
    def __init__(self):
        super().__init__("yolo26_pose_keypoints_node")

        # Parameters let you override every camera/runtime value from launch or
        # config/zed.yaml without editing this file. Defaults reproduce
        # the original hard-coded behavior exactly.
        # -- detector / inference --
        self.declare_parameter("engine_path", ENGINE_PATH)
        self.declare_parameter("threshold", DEFAULT_THRESHOLD)
        self.declare_parameter("nms_iou", DEFAULT_NMS_IOU)
        self.declare_parameter("publish_debug_every", DEFAULT_PUBLISH_DEBUG_EVERY)
        self.declare_parameter("debug_scores", True)
        # Per-frame JSON keypoint publish (/detector/armors_keypoints_json) is a
        # DEBUG-only mirror of the typed ArmorKeypointArray. Nothing in the autoaim
        # pipeline subscribes to it; json.dumps every frame is pure CPU waste at the
        # grab rate. Default OFF (WORKLOG §4f stage 1). Turn on only for external
        # JSON tooling.
        self.declare_parameter("publish_keypoint_json", False)
        # Software-pipeline the inference: run the CPU postprocess+publish of frame
        # N-1 WHILE the GPU computes frame N (no GPU/CPU overlap otherwise). Hides
        # the ~8 ms CPU stage under the ~15 ms inference → higher throughput. Costs
        # ONE frame of extra latency, which the node's latency-aware prediction
        # already accounts for (the capture timestamp is preserved). WORKLOG §4f
        # stage 3. Set False to restore the strictly-serial loop.
        self.declare_parameter("pipeline_inference", True)
        # -- camera capture / runtime (previously hard-coded constants) --
        self.declare_parameter("resolution", DEFAULT_RESOLUTION)
        self.declare_parameter("fps", DEFAULT_FPS)
        self.declare_parameter("image_flip", DEFAULT_IMAGE_FLIP)
        self.declare_parameter("auto_exposure", DEFAULT_AUTO_EXPOSURE)
        self.declare_parameter("exposure", DEFAULT_EXPOSURE)
        self.declare_parameter("gain", DEFAULT_GAIN)
        self.declare_parameter("auto_white_balance", DEFAULT_AUTO_WHITE_BALANCE)
        self.declare_parameter("camera_info_every", DEFAULT_CAMERA_INFO_EVERY)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("imu_frame_id", DEFAULT_IMU_FRAME_ID)

        self.engine_path = self.get_parameter("engine_path").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.nms_iou = float(self.get_parameter("nms_iou").value)
        self.publish_debug_every = int(self.get_parameter("publish_debug_every").value)
        self.debug_scores = bool(self.get_parameter("debug_scores").value)
        self.publish_keypoint_json = bool(self.get_parameter("publish_keypoint_json").value)
        self.pipeline_inference = bool(self.get_parameter("pipeline_inference").value)

        res_name = str(self.get_parameter("resolution").value).upper()
        if res_name not in ZED_RESOLUTIONS:
            raise RuntimeError(
                f"Unknown ZED resolution '{res_name}'. Supported: {sorted(ZED_RESOLUTIONS)}"
            )
        self.resolution_name = res_name
        self.native_w, self.native_h = ZED_RESOLUTIONS[res_name]
        self.fps = int(self.get_parameter("fps").value)
        self.image_flip = bool(self.get_parameter("image_flip").value)
        self.auto_exposure = bool(self.get_parameter("auto_exposure").value)
        self.exposure = int(self.get_parameter("exposure").value)
        self.gain = int(self.get_parameter("gain").value)
        self.auto_white_balance = bool(self.get_parameter("auto_white_balance").value)
        self.camera_info_every = int(self.get_parameter("camera_info_every").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)

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

        # Letterbox geometry for native (per-eye) -> square engine input.
        self.scale_net_to_native = float(self.native_w) / float(self.img_size)
        self.unscaled_h_net = int(round(float(self.native_h) / self.scale_net_to_native))
        self.pad_y_net = int((self.img_size - self.unscaled_h_net) // 2)

        # self.get_logger().info(
        #     f"Preprocess: {self.native_w}x{self.native_h} -> {self.img_size}x{self.img_size}, "
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
        self.d_rgba = cuda.mem_alloc(self.native_w * self.native_h * 4)
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
        params.camera_resolution = getattr(sl.RESOLUTION, self.resolution_name)
        params.camera_fps = self.fps
        params.depth_mode = sl.DEPTH_MODE.NONE
        params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        # Image flip (camera mounted upside-down). FLIP_MODE.ON also swaps which
        # physical sensor produces the "left image" (see INSTRUCTIONS.md).
        params.camera_image_flip = sl.FLIP_MODE.ON if self.image_flip else sl.FLIP_MODE.OFF

        err = self.zed.open(params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Could not open ZED camera: {err}")

        # Exposure / gain / white balance for dark arenas. Tune on the real field
        # via config/zed.yaml. EXPOSURE/GAIN are only applied in manual
        # mode (auto_exposure False); otherwise AEC_AGC drives them.
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1 if self.auto_exposure else 0)
        if not self.auto_exposure:
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, self.exposure)
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, self.gain)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1 if self.auto_white_balance else 0)

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

        # Cache the CameraInfo message: intrinsics never change at runtime, and
        # the old code called zed.get_camera_information() EVERY FRAME at 120 Hz.
        self.cam_info_msg = CameraInfo()
        self.cam_info_msg.header.frame_id = self.frame_id
        self.cam_info_msg.width = self.native_w
        self.cam_info_msg.height = self.native_h
        self.cam_info_msg.k = [
            calib.fx, 0.0, calib.cx,
            0.0, calib.fy, calib.cy,
            0.0, 0.0, 1.0,
        ]
        self.cam_info_msg.d = [0.0, 0.0, 0.0, 0.0]
        self.cam_info_msg.distortion_model = "plumb_bob"

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
                "frame_id": self.frame_id,
            },
            "image_width": self.native_w,
            "image_height": self.native_h,
            "keypoint_order": KEYPOINT_NAMES,
            "detections": detections,
        }
        return json.dumps(payload, separators=(",", ":"))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _process_and_publish(self, output, now_msg):
        """Decode one inference output and publish the detection/keypoint topics.
        Extracted from loop() so it can run for frame N-1 while the GPU computes
        frame N (software pipeline, WORKLOG §4f stage 3). Pure CPU + ROS publish;
        no CUDA context needed. IMU / camera_info / debug image stay in loop()."""
        det_array_msg = Detection2DArray()
        det_array_msg.header.stamp = now_msg
        det_array_msg.header.frame_id = self.frame_id

        kpt_array_msg = ArmorKeypointArray()
        kpt_array_msg.header.stamp = now_msg
        kpt_array_msg.header.frame_id = self.frame_id
        kpt_array_msg.image_width = self.native_w
        kpt_array_msg.image_height = self.native_h
        kpt_array_msg.keypoint_order = KEYPOINT_NAMES

        json_detections: List[Dict] = []

        if self.output_mode == "post_nms":
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
                    L = max(0.0, min(float(self.native_w), float(L_r)))
                    T = max(0.0, min(float(self.native_h), float(T_r)))
                    R = max(0.0, min(float(self.native_w), float(L_r + W_r)))
                    B = max(0.0, min(float(self.native_h), float(T_r + H_r)))
                    W = R - L
                    H = B - T
                    if W <= 1.0 or H <= 1.0:
                        continue

                    cls = int(v_classes[raw_i])
                    conf = float(v_confs[raw_i])
                    kp = kpts[raw_i].copy()
                    kp[:, 0] = np.clip(kp[:, 0], 0.0, float(self.native_w))
                    kp[:, 1] = np.clip(kp[:, 1], 0.0, float(self.native_h))

                    # Legacy bbox topic for current autoaim compatibility.
                    det = Detection2D()
                    det.header.stamp = now_msg
                    det.header.frame_id = self.frame_id
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
                    kmsg.header.frame_id = self.frame_id
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

                    if self.publish_keypoint_json:
                        json_detections.append({
                            "class_id": cls,
                            "class_name": CLASS_NAMES.get(cls, str(cls)),
                            "confidence": conf,
                            "bbox": {
                                "left": float(L), "top": float(T),
                                "right": float(R), "bottom": float(B),
                                "cx": float(L + W * 0.5), "cy": float(T + H * 0.5),
                                "w": float(W), "h": float(H),
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
        if self.publish_keypoint_json:  # WORKLOG §4f stage 1: off by default (unused, CPU waste)
            self.kpt_json_pub.publish(String(data=self._make_keypoint_json(now_msg, json_detections)))

    def loop(self):
        # ── Per-stage latency ledger (WORKLOG §4f — framerate profiling) ──
        # The loop is fully serial: grab (blocks for the next frame) → infer
        # (stream.synchronize blocks) → CPU postprocess → publish, with NO
        # GPU/CPU overlap. At 44 Hz (≈23 ms) a fast spinner is under-sampled, so
        # the single biggest lever is finding which stage eats the 23 ms. These
        # EMAs are logged every ~2 s so the bottleneck is visible live.
        prof = {"grab": 0.0, "infer": 0.0, "post": 0.0, "pub": 0.0, "total": 0.0}
        prof_a = 0.05  # EMA weight
        t_loop_prev = None
        pending = None  # (output_host_copy, now_msg) of the in-flight GPU frame (pipeline)
        while self.running:
            t_s0 = time.perf_counter()
            if self.zed.grab(sl.RuntimeParameters()) != sl.ERROR_CODE.SUCCESS:
                continue

            # CAPTURE timestamp from the ZED SDK, not "now": the autoaim node
            # interpolates the gimbal angles at this stamp for the camera->odom
            # projection. sl.TIME_REFERENCE.IMAGE is the moment the frame was
            # captured (epoch ns, same clock domain as the node clock when
            # use_sim_time is off). Using get_clock().now() here would hide the
            # capture->grab latency from the angle sync.
            t_img_ns = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            if t_img_ns > 0:
                now_msg = RclpyTime(nanoseconds=t_img_ns).to_msg()
            else:
                now_msg = self.get_clock().now().to_msg()
            self.zed.retrieve_image(self.mat_cpu, sl.VIEW.LEFT, sl.MEM.CPU)
            t_s1 = time.perf_counter()  # grab+retrieve done

            # TensorRT inference.
            self.cuda_ctx.push()
            cuda.memcpy_htod_async(self.d_rgba, self.mat_cpu.get_data(), self.stream)
            self.pre_fn(
                self.d_rgba,
                self.d_in,
                np.int32(self.native_w),
                np.int32(self.native_h),
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
            self.cuda_ctx.pop()

            # ── Software pipeline (WORKLOG §4f stage 3) ──
            # While the GPU computes THIS frame, run the CPU postprocess+publish of
            # the PREVIOUS frame (pending). The detection output is published one
            # frame late, but its CAPTURE timestamp is preserved so the node's
            # latency-aware prediction compensates. Set pipeline_inference:=false to
            # restore the strictly-serial loop.
            t_p0 = time.perf_counter()
            if self.pipeline_inference:
                if pending is not None:
                    self._process_and_publish(pending[0], pending[1])
                t_p1 = time.perf_counter()
                self.cuda_ctx.push()
                self.stream.synchronize()
                self.cuda_ctx.pop()
                t_s2 = time.perf_counter()
                # snapshot THIS frame's output (host copy) for next iteration's CPU stage
                pending = (self._raw_output_as_rows().copy(), now_msg)
            else:
                self.cuda_ctx.push()
                self.stream.synchronize()
                self.cuda_ctx.pop()
                t_s2 = time.perf_counter()
                self._process_and_publish(self._raw_output_as_rows(), now_msg)
                t_p1 = time.perf_counter()
            post_ms = (t_p1 - t_p0) * 1e3
            gpu_wait_ms = ((t_s2 - t_p1) if self.pipeline_inference else (t_s2 - t_s1)) * 1e3

            # ── Latency ledger update + throttled log (WORKLOG §4f) ──
            # In pipeline mode "gpu_wait" is the GPU time NOT hidden under the CPU
            # stage (≈0 if CPU-bound); "post" is the CPU postprocess+publish time;
            # "loop" + Hz is the real throughput.
            prof["grab"]  += prof_a * ((t_s1 - t_s0) * 1e3 - prof["grab"])
            prof["infer"] += prof_a * (gpu_wait_ms - prof["infer"])
            prof["post"]  += prof_a * (post_ms - prof["post"])
            if t_loop_prev is not None:
                prof["total"] += prof_a * ((t_s0 - t_loop_prev) * 1e3 - prof["total"])
            t_loop_prev = t_s0
            if self.frame_count % 90 == 0:
                rate = 1000.0 / prof["total"] if prof["total"] > 1e-6 else 0.0
                self.get_logger().info(
                    "LATENCY[ms] grab=%.1f gpu_wait=%.1f post=%.1f | loop=%.1f (%.0f Hz)%s"
                    % (prof["grab"], prof["infer"], prof["post"],
                       prof["total"], rate, " [pipe]" if self.pipeline_inference else ""))

            # IMU every frame (ZED-only feature). CameraInfo throttled: the
            # autoaim node latches intrinsics on the first message, so there is
            # no need to resend at the full grab rate (camera_info_every).
            self._publish_imu(now_msg)
            if self.camera_info_every > 0 and self.frame_count % self.camera_info_every == 0:
                self._publish_camera_info(now_msg)

            # Raw image for the viewer node. Publish less often to reduce bandwidth.
            self.frame_count += 1
            if self.publish_debug_every > 0 and self.frame_count % self.publish_debug_every == 0:
                img_msg = self.bridge.cv2_to_imgmsg(self.mat_cpu.get_data(), "bgra8")
                img_msg.header.stamp = now_msg
                img_msg.header.frame_id = self.frame_id
                self.img_pub.publish(img_msg)

    # ------------------------------------------------------------------
    # IMU publisher: same logic as your bbox-only node.
    # ------------------------------------------------------------------
    def _publish_imu(self, now_msg):
        # BUGFIX: the get_sensors_data() call was commented out, so accel/gyro/
        # orientation were read from a default-constructed (EMPTY) SensorsData —
        # /zed/imu_data was publishing garbage. Restored with silent early-return
        # guards (no log spam at 120 Hz).
        sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            return

        imu_data = sensors_data.get_imu_data()
        if not imu_data.is_available:
            return

        accel = np.array(imu_data.get_linear_acceleration())
        gyro = np.array(imu_data.get_angular_velocity())
        gyro_rad = np.deg2rad(gyro)

        q_curr = np.array(imu_data.get_pose().get_orientation().get())
        norm = float(np.linalg.norm(q_curr))
        if abs(norm - 1.0) > 0.1:
            return  # orientation not yet valid

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
        imu_msg.header.frame_id = self.imu_frame_id
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
        # Cached at init — intrinsics don't change; avoids an SDK call per frame.
        self.cam_info_msg.header.stamp = now_msg
        self.cam_info_pub.publish(self.cam_info_msg)

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
