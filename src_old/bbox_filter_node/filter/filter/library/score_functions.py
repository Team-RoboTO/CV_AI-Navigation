from vision_msgs.msg import Detection2D
from math import sqrt


# Compute score according to the distance from center of camera display
def _get_centered(detection: Detection2D):
    if(detection is None):
        print("i can't find best bbox")
    x = detection.bbox.center.position.x
    y = detection.bbox.center.position.y
    dist = sqrt((x - 640/2) ** 2 + (y - 480/2) ** 2)
    score = -dist/1000
    print(f'distance from center score: {score}')
    return score


# Compute score according to the area of the bbox
def _get_wide(detection: Detection2D):
    if(detection is None):
        print("i can't find best bbox")
    width = detection.bbox.size_x  # width
    height = detection.bbox.size_y # height
    wide = width * height
    score = wide/(640*480/40)
    print(f'wide score: {score}')
    return score


# Compute score according to the spatial distance from the bbox
def _get_close(detection: Detection2D):
    if(detection is None):
        print("i can't find best bbox")
    pos_x = detection.results[0].pose.pose.position.x # x
    pos_y = detection.results[0].pose.pose.position.y # y
    pos_z = detection.results[0].pose.pose.position.z # z
    # print(f'x: {pos_x}')
    # print(f'y: {pos_y}')
    # print(f'z: {pos_z}')
    dist = sqrt(pos_x ** 2 + pos_y ** 2 + pos_z ** 2) # distance from sensor
    score = -dist/7
    print(f'distance score: {score}')
    return score


def _get_fract_sizes(detection: Detection2D):
    if(detection is None):
        print("i can't find best bbox")
    width = detection.bbox.size_x  # width
    height = detection.bbox.size_y # height
    score =  width / height / 5
    print(f'fraction score: {score}')
    return score
    