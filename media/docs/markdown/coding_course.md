# Coding Course

## Goal Of This Course

This chapter teaches the minimum coding knowledge needed to read, run, and
debug this package. It does not try to teach all of C++ or all of ROS 2. It
teaches the ideas that appear in this repository:

- source files and header files
- variables, functions, structs, and classes
- loops and conditions
- ROS 2 nodes, topics, messages, publishers, and subscribers
- CMake and package metadata
- practical debugging habits

## What A Program Is

A program is a set of instructions for the computer. Source code is the human
readable text that describes those instructions. A compiler translates C++
source code into machine code that the computer can run.

This package is a ROS 2 package named `auto_aim`. ROS 2 is a robotics
framework. It lets separate programs communicate by sending messages on named
channels called topics.

## Files, Folders, And Builds

The important hand-written code is under `src/`. Generated files under
`build/` and `install/` are build output. New members should usually read and
edit `src/`, not generated output.

| Folder or file | Beginner explanation |
| --- | --- |
| `src/src/*.cpp` | C++ implementation files. These contain function bodies: what the code actually does. |
| `src/include/auto_aim/*.hpp` | C++ header files. These declare the types and functions other files can use. |
| `src/msg/*.msg` | ROS message definitions. ROS generates C++ code from these during the build. |
| `src/config/*.yaml` | Parameter values loaded at runtime. This is where robot-specific tuning belongs. |
| `src/launch/*.py` | Launch scripts that start the node with parameters. |
| `src/CMakeLists.txt` | Build instructions for CMake and colcon. |
| `src/package.xml` | ROS package metadata and dependencies. |

Typical build and run commands from the workspace root:

```bash
colcon build --packages-select auto_aim
source install/setup.bash
ros2 launch auto_aim auto_aim_realsense_16.launch.py
```

If you edit C++ or message files, rebuild. If you only edit YAML parameters,
you usually only need to relaunch.

## C++ Source And Header Files

C++ projects often split code into headers and source files.

- A header file, such as `tracker.hpp`, says what a class or function looks
  like.
- A source file, such as `tracker.cpp`, contains the implementation.

Example from the tracker header:

```cpp
class Tracker
{
public:
  explicit Tracker(const TrackerConfig & cfg);
  void update(const std::vector<ArmorDetection> & detections, double dt);
  AimResult computeAim(double current_yaw, double current_pitch,
                       double override_pred_lead_s = -1.0) const;
};
```

This says a `Tracker` has a constructor, an `update` function, and a
`computeAim` function. The detailed math lives in `tracker.cpp`.

## Includes

At the top of a C++ file, `#include` brings in declarations from another file
or library:

```cpp
#include <rclcpp/rclcpp.hpp>
#include "auto_aim/tracker.hpp"
```

Angle brackets usually mean an external library or system header. Quotes
usually mean a header from this package.

## Namespaces

The code is wrapped in:

```cpp
namespace auto_aim
{
  // code here
}
```

A namespace is a named container that prevents name collisions. The class is
`auto_aim::Tracker`, not just `Tracker`, when referred to from outside the
namespace.

## Variables And Types

A variable stores a value. A type tells C++ what kind of value it stores.

```cpp
double bullet_speed = 25.0;
bool tracking = false;
int confirm_frames = 3;
std::string class_id = "0";
```

Common types in this repo:

| Type | Meaning |
| --- | --- |
| `double` | Floating point number, used for math. |
| `float` | Smaller floating point number, often used in ROS messages. |
| `int` | Whole number. |
| `bool` | True or false. |
| `std::string` | Text. |
| `std::vector<T>` | Resizable list of values of type `T`. |
| `std::array<T,N>` | Fixed-size list with exactly `N` values. |
| `Eigen::VectorXd` | Math vector from the Eigen library. |
| `cv::Mat` | Matrix/image container from OpenCV. |

## Structs

A struct groups related values under one name. For example:

```cpp
struct ArmorDetection
{
  double x, y, z;
  double yaw;
  std::string class_id;
  double confidence;
};
```

Instead of passing six separate values everywhere, the code passes one
`ArmorDetection`.

Important structs in this package:

- `TrackerConfig`: all tracker, ballistic, and gate parameters.
- `ArmorDetection`: one transformed detection in odom frame.
- `AimResult`: output from the aim planner.
- `DebugFrame`: internal per-frame debug data before publishing.
- `PnPResult`: output from the PnP solver.

## Functions

A function is reusable code with a name. It may take inputs and return an
output.

```cpp
double Tracker::targetRange() const
{
  return std::sqrt(x_(0)*x_(0) + x_(2)*x_(2) + x_(4)*x_(4));
}
```

This function returns a `double`. The `const` at the end means it does not
modify the `Tracker` object.

## Classes

A class groups data and functions together. In this repo, each major pipeline
piece is a class or stateless helper:

| Class or helper | Responsibility |
| --- | --- |
| `AutoAimNode` | ROS wiring: parameters, subscriptions, callbacks, publishers. |
| `PnPSolver` | 2D box to 3D armor pose. |
| `FrameTransformer` | Camera frame to odom frame. |
| `Tracker` | EKF, target association, aiming, and ballistics. |
| `FireGate` | Fire/hold decision and reason. |
| `DetectionAdapter` | Convert detector messages into candidates. |
| `DebugPublisher` | Publish structured debug messages. |
| `ConfigValidator` | Validate parameters at startup. |

## Public And Private

Classes often have `public` and `private` sections.

- `public`: other code is allowed to call these functions or read these
  members.
- `private`: internal implementation details. Other code should not depend on
  them.

For example, other code can call `tracker_->update(...)`, but it cannot
directly change `x_`, the EKF state. That keeps the tracker responsible for
its own consistency.

## Pointers And Unique Ownership

Some objects are stored through pointers:

```cpp
std::unique_ptr<Tracker> tracker_;
tracker_ = std::make_unique<Tracker>(cfg);
```

A pointer stores the location of an object. `std::unique_ptr` means one owner
is responsible for that object. When the owner is destroyed, the object is
automatically cleaned up.

To call a function through a pointer, use `->`:

```cpp
tracker_->update(armors, dt);
```

## References And `const`

You will often see parameters like:

```cpp
void update(const std::vector<ArmorDetection> & detections, double dt);
```

`&` means pass by reference. The function can use the original object without
copying the whole vector. `const` means the function promises not to modify
that vector.

This is common for performance and safety.

## Conditions

`if` chooses between code paths:

```cpp
if (!pnp_.ready()) return;
if (!imu_valid_) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
    "Waiting for /micro_imu");
  return;
}
```

The `return` exits the current function early. This pattern is called a guard
clause. It keeps the rest of the function from running when required inputs are
missing.

## Loops

A loop repeats work. This package often loops over detections or four armor
faces:

```cpp
for (const auto & det : detections) {
  // use det
}
```

`const auto & det` means: for each detection, call it `det`, do not copy it,
and do not modify it.

## ROS 2 Nodes

A ROS 2 node is a running component that communicates with other nodes. The
`AutoAimNode` class inherits from `rclcpp::Node`:

```cpp
class AutoAimNode : public rclcpp::Node
```

This gives it ROS features such as parameters, subscriptions, publishers, and
logging.

## Topics And Messages

A topic is a named channel. A message type defines what data is sent on that
channel.

| Topic | Message type | Direction |
| --- | --- | --- |
| `/detector/armors_keypoints` | `auto_aim/ArmorKeypointArray` | input primary |
| `/detector/armors` | `vision_msgs/Detection2DArray` | input fallback |
| `/camera_info` | `sensor_msgs/CameraInfo` | input |
| `/micro_imu` | `std_msgs/Float32MultiArray` | input |
| `/tracker/cmd_gimbal` | `geometry_msgs/Twist` | output |
| `/cmd_vel_AI` | `geometry_msgs/Twist` | output legacy |
| `/tracker/aim_pixels` | `geometry_msgs/Twist` | output debug |
| `/tracker/marker` | `visualization_msgs/MarkerArray` | output debug |
| `/auto_aim/debug` | `auto_aim/AutoAimDebug` | output debug |

## Subscriptions

A subscription tells ROS: when a message arrives on this topic, call this
function.

```cpp
kpt_sub_ = create_subscription<auto_aim::msg::ArmorKeypointArray>(
  keypoint_topic_, rclcpp::SensorDataQoS(),
  std::bind(&AutoAimNode::keypointCallback, this, std::placeholders::_1));
```

This means `keypointCallback` runs once for each incoming YOLOv26-pose
detection frame. When `use_keypoints=false`, the node instead subscribes to
`/detector/armors` and runs the bbox fallback `detectionCallback`.

## Publishers

A publisher sends messages:

```cpp
cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
  "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());

geometry_msgs::msg::Twist cmd;
cmd.angular.z = yaw_target_micro;
cmd.angular.y = pitch_target_micro;
cmd.angular.x = fire_decision ? 1.0 : 0.0;
cmd_pub_->publish(cmd);
```

The firmware contract depends on these exact fields. Do not move yaw, pitch,
fire, or distance to different fields without a matching firmware change.

## Parameters

Parameters are runtime settings. The node declares them at startup:

```cpp
cfg.bullet_speed = declare_parameter("bullet_speed", 25.0);
```

If a YAML file provides `bullet_speed`, ROS uses the YAML value. If not, the
default `25.0` is used.

Robot-specific values belong in `src/config/params_realsense_16.yaml` or
`src/config/params_zed_64.yaml`, not hard-coded into C++.

## Launch Files

Launch files start nodes and provide parameters. For example,
`auto_aim_realsense_16.launch.py` loads
`config/params_realsense_16.yaml`. Use launch files for normal robot runs
because they keep the startup command consistent.

## Logging

ROS logging prints useful runtime information:

```cpp
RCLCPP_INFO(get_logger(), "Auto-aim node started");
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "PnP FAILED");
```

Throttled logs print at most once per time window. This prevents the terminal
from being flooded at 30 frames per second.

## External Libraries Used Here

| Library | Why this package uses it |
| --- | --- |
| ROS 2 `rclcpp` | Nodes, parameters, publishers, subscribers, logging. |
| OpenCV | PnP, camera matrices, Rodrigues rotations, point projection. |
| Eigen | Linear algebra for EKF state, matrices, covariance, and Kalman math. |
| `tf2` | Quaternions, vector rotations, marker orientations. |
| `angles` | Correct wrap-around math for radians near `+pi` and `-pi`. |

## How To Read A New File

Use this routine when opening a file for the first time:

1. Read the matching header first if one exists.
2. Identify the main data types: structs and classes.
3. Find the public functions. These are the file's interface.
4. Find where those functions are called with `rg "functionName"`.
5. Read implementation from the top-level function inward.
6. Track units: meters, radians, pixels, seconds.
7. Track frame: camera, gimbal body, or odom.

## How To Make A Small Change Safely

1. State the behavior you want to change in one sentence.
2. Find the earliest pipeline stage responsible for that behavior.
3. Change only that module when possible.
4. Rebuild with `colcon build --packages-select auto_aim`.
5. Relaunch and record debug data.
6. Compare `/auto_aim/debug` before and after.

Avoid changing several unrelated parameters at once. If the behavior changes,
you will not know which change caused it.

## Useful Terminal Commands

| Command | Purpose |
| --- | --- |
| `rg "text" src` | Search the source tree quickly. |
| `rg --files src` | List source files. |
| `colcon build --packages-select auto_aim` | Build only this package. |
| `source install/setup.bash` | Load the built package into the current shell. |
| `ros2 topic list` | Show active ROS topics. |
| `ros2 topic echo /auto_aim/debug` | Print debug messages. |
| `ros2 topic hz /detector/armors` | Measure detector publish rate. |
| `ros2 param list /auto_aim` | Show parameters for the node. |
| `ros2 bag record ...` | Record data for offline debugging. |

## Common Beginner Mistakes

- Editing generated files under `build/` or `install/`. Edit `src/` instead.
- Forgetting to rebuild after editing C++ or message files.
- Forgetting to run `source install/setup.bash` after a build.
- Mixing degrees and radians.
- Mixing camera-frame and odom-frame coordinates.
- Hiding calibration mistakes with `pitch_offset_deg` or `yaw_offset_deg`.
- Changing the `/tracker/cmd_gimbal` field mapping without a firmware update.

## First Exercises

1. Find where `bullet_speed` is declared in the C++ node.
2. Find where `/tracker/cmd_gimbal` is created and where messages are
   published on it.
3. Find all files that mention `FireGate`.
4. Change only a YAML value, relaunch, and confirm the parameter changed with
   `ros2 param get`.
5. Record `/auto_aim/debug` while pointing at a static target and identify
   which fields belong to detection, PnP, EKF, aim, command, and fire gate.
