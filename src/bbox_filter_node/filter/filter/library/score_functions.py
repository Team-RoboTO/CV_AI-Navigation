from vision_msgs.msg import Detection2D
from math import sqrt

# --- Ideal Aspect Ratios for Robomaster Armor Plates ---
# Small armor: ~55mm height, 135mm width -> aspect ratio ~2.45
# Large armor: ~55mm height, 225mm width -> aspect ratio ~4.09
# We can use these as a reference.
IDEAL_ASPECT_RATIO_SMALL = 2.45
IDEAL_ASPECT_RATIO_LARGE = 4.09

def _get_centered(detection: Detection2D, image_width: int, image_height: int) -> float:
    """
    Computes a score based on the detection's distance from the image center.
    A lower distance results in a higher score (closer to 0).
    The score is negative, ranging from -1 (at corner) to 0 (at center).
    """
    x = detection.bbox.center.position.x
    y = detection.bbox.center.position.y
    center_x = image_width / 2.0
    center_y = image_height / 2.0

    dist = sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
    max_dist = sqrt(center_x**2 + center_y**2)
    
    # Normalize to a range of [-1, 0]
    score = -dist / max_dist
    return score


def _get_wide(detection: Detection2D, image_width: int, image_height: int) -> float:
    """
    Computes a score based on the bounding box area.
    A larger area results in a higher score.
    The score is positive, ranging from 0 to 1.
    """
    width = detection.bbox.size_x  # width
    height = detection.bbox.size_y # height
    wide = width * height
    max_area = image_width * image_height
    
    # Normalize to a range of [0, 1]
    score = wide / max_area
    return score



def _get_fract_sizes(detection: Detection2D) -> float:
    """
    Computes a score based on the aspect ratio (width/height) of the bbox.
    The score is higher if the aspect ratio is close to that of a typical
    Robomaster armor plate.
    Score is in range [0, 1].
    """
    width = detection.bbox.size_x  # width
    height = detection.bbox.size_y # height

    if height == 0:
        return 0.0

    ratio = width / height

    # Calculate how close the ratio is to either ideal ratio
    error_small = abs(ratio - IDEAL_ASPECT_RATIO_SMALL) / IDEAL_ASPECT_RATIO_SMALL
    error_large = abs(ratio - IDEAL_ASPECT_RATIO_LARGE) / IDEAL_ASPECT_RATIO_LARGE

    # Take the smaller error, as it could be either a small or large plate
    min_error = min(error_small, error_large)

    # Convert error to score. 1 for perfect match, 0 for high error.
    # We use an exponential decay to penalize deviations more sharply.
    score = (1.0 - min_error) ** 2 if min_error <= 1.0 else 0.0
    return score
    