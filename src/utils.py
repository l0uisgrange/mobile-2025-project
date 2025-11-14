import cv2
import numpy as np

def show_grid(frame: np.ndarray, grid: np.ndarray):
    """
    Shows the grid inside a new window

    :param frame: The raw image frame
    :param grid: The grid
    """
    combined = np.hstack((frame, grid))
    cv2.imshow("Image", combined)