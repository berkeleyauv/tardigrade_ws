import cv2
import numpy as np

from tardigrade_perception.gate_detector import find_gate_posts


def test_gate_posts_are_detected_from_pixels():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (180, 80), (205, 300), (0, 100, 255), -1)
    cv2.rectangle(image, (435, 80), (460, 300), (0, 100, 255), -1)
    boxes = find_gate_posts(image)
    assert boxes is not None
    assert boxes[0][0] < boxes[1][0]


def test_single_post_is_not_a_gate():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (180, 80), (205, 300), (0, 100, 255), -1)
    assert find_gate_posts(image) is None
