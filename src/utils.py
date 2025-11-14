import cv2
import numpy as np

from src.consts import *


def draw_control_room(frame: np.ndarray):
    """
    Shows the grid inside a new window

    :param frame: The raw image frame
    """
    cv2.imshow(WINDOW_NAME, frame)
