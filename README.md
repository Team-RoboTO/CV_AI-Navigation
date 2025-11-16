# CV_AI-Navigation
Below the guide for setting up your git environment in your local machine and the instructions for the PR opening (Pull Request).
This repository doesn't allow you to merge automatically your work on the main, you have to open a PR and wait for the review of them.
When a PR is approved and merged on the main it create a tag, in that way we can follow the evolution of the Repo and come to a previous version ifneeded.

# Contributing to CV_AI & Navigation
First step clone the repo, go inside the root of the folder click on the button code, and copy the https link.
Clone it in your local machine "git clone {link}"

## Contribution Workflow

### 1. Create a branch for your feature

```bash (on your terminal)
git checkout main
git pull origin main
git checkout -b feature/descriptive-name
```

### 2. Work and commit

```bash
git add {name of the file modified}
git commit -m "Clear description" -> for example "CV_AI-Bbox_target-Feature for better selection"
```

### 3. Before opening a PR

```bash
# Update with latest main
git fetch origin main
git rebase origin/main

# Resolve any conflicts
# Then push
git push origin feature/descriptive-name 
```

### 4. Open Pull Request

**⚠️ IMPORTANT:** The title must contain the version!

```
[vX.Y.Z] {Sub-Division}_{Specification} Feature description
```
Sub Division:
    CV -> related with the ROS2 pipeline
    AI -> related with the AI models/ realsense/ Zed code
    Navigation -> SLAM algorithms
Specification
    Bbox_Shooting
    Uart protocol
    SLAM
    Sensors
    AI Model
    Camera

**Examples:**
- `[v1.0.1] AI_Camera Fix camera calibration bug`
- `[v1.0.2] Navigation_Sensors Add LiDAR sensor support`
- `[v1.0.3] Navigation_SLAM Rewrite navigation algorithm`

### 5. Approval and Merge
Open a PR via github UI or through gh plugin from terminal
Example:

gh pr create --title [v1.0.1] AI_Camera Fix camera calibration bug`
Or when you open a PR from UI remeber to respect what written in this guide for creating a title with the correct wording

- PR requires approval from `Boskyx`
- After approval, merge is performed
- Tag and release are **automatically** created!

## 🏷️ Releases

All releases are available here:
https://github.com/Team-RoboTO/CV_AI-Navigation/releases

To return to a specific version:
```bash
git checkout v1.2.3
```

## ⚠️ Important Rules

1. ❌ **NEVER** push directly to `main` (it's protected)
2. ✅ **ALWAYS** open PR with version in title
3. ✅ **ALWAYS** rebase before pushing
4. ✅ **ALWAYS** wait for approval before merging
5. ✅ Each version is **unique** (don't reuse v1.2.3!)

