# Auto Aim Onboarding Course

Mathematics, Coding, and Source Walkthrough

Date: May 10, 2026

## How To Use This Course

This document is written for new members who may have no previous coding
experience. It is not only a reference manual. It is meant to be read as a
course:

1. Read the mathematics course first. The code will make more sense once the
   coordinate frames, prediction, and filtering ideas are familiar.
2. Read the coding course next. It introduces C++, ROS 2, and the build system
   using this repository as the example.
3. Use the code walkthrough while opening the source files in an editor. Read
   one file at a time and compare the prose with the implementation.
4. Debug with the structured topic `/auto_aim/debug`. Most failures can be
   located by following the same order as the pipeline.

The most important rule for this repository is simple: do not tune a downstream
parameter to hide an upstream error. For example, do not change
`pitch_offset_deg` to compensate for an unmeasured bullet speed or an incorrect
barrel offset. Fix the earliest wrong stage first.

## Markdown Files

- [math_course.md](math_course.md): mathematics course for the auto-aim
  pipeline.
- [coding_course.md](coding_course.md): beginner coding and ROS 2 course.
- [code_walkthrough.md](code_walkthrough.md): source-by-source package
  walkthrough.

## LaTeX Source

The matching LaTeX files are in `src/docs/latex/`.

To compile the LaTeX document from the workspace root:

```bash
cd src/docs/latex
pdflatex main.tex
pdflatex main.tex
```

Running `pdflatex` twice updates the table of contents.

## Repository Map

| Path | Purpose |
| --- | --- |
| `src/src/auto_aim_node.cpp` | The ROS 2 node. Owns subscriptions, publishers, parameters, callbacks, command smoothing, and debug publication. |
| `src/src/pnp_solver.cpp` | Converts a 2D detector bounding box into a 3D pose using OpenCV PnP. |
| `src/src/frame_transformer.cpp` | Converts camera-frame PnP output into the odom/microcontroller reference frame. |
| `src/src/tracker.cpp` | Extended Kalman filter, target association, state machine, face prediction, and ballistic aim planning. |
| `src/src/fire_gate.cpp` | Central fire/hold decision logic and blocker reason generation. |
| `src/src/detection_adapter.cpp` | Converts raw detector messages into small validated detection candidates. |
| `src/src/debug_publisher.cpp` | Copies per-frame debug data into the `/auto_aim/debug` ROS message and logs blocker histograms. |
| `src/src/config_validator.cpp` | Checks startup parameters for impossible or suspicious values. |
| `src/include/auto_aim/*.hpp` | Public declarations for the C++ modules. |
| `src/msg/AutoAimDebug.msg` | Structured debug message for one frame. |
| `src/msg/GimbalCmd.msg` | Placeholder message, not used at runtime. |
| `src/config/*.yaml` | Robot-specific parameter files. |
| `src/launch/*.py` | ROS 2 launch files. |
| `src/docs/*.md` | Existing operational guides. |
