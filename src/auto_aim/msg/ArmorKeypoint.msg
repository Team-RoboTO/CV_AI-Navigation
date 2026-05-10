std_msgs/Header header
int32 class_id
string class_name
float32 confidence
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h

# Fixed corner convention for YOLO-pose labels and PnP:
# keypoints_xy = [TLx, TLy, TRx, TRy, BRx, BRy, BLx, BLy] in image pixels.
float32[8] keypoints_xy
float32[4] keypoint_scores
bool[4] keypoint_valid
