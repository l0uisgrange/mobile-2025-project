from typing import Any

import cv2
import numpy as np
from cv2 import Mat
from numpy import ndarray, dtype, integer, floating

from src.consts import GRID_SHAPE


def list_cameras() -> list[int]:
    """
    Lists all available cameras on the system.

    :return: A list of integers
    """
    index = 0
    arr = []
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            break
        else:
            arr.append(index)
        cap.release()
        index += 1
    return arr


def start_camera(index: int = 0) -> cv2.VideoCapture:
    """
    Starts the video capture for a given camera index.

    :returns: The video capture object.
    """
    cap = cv2.VideoCapture(index)
    return cap


def get_grid(cap: cv2.VideoCapture) -> tuple[ndarray, ndarray] | None:
    """
    Reads a frame from the given video capture and returns the grid.

    :param cap: Video capture object.

    :returns: Scene grid with obstacles
    """
    ret, frame = cap.read()

    if not ret:
        return None

    # Filtering
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_filtered = cv2.bilateralFilter(frame_gray, 15, 7, 8)

    # Treshhold
    _, frame_tresh = cv2.threshold(frame_filtered, 200, 255, cv2.THRESH_BINARY_INV)

    # Build grid
    rows, cols = GRID_SHAPE
    cell_height, cell_width = frame_tresh.shape
    grid = np.zeros((rows, cols), dtype=int)
    for r in range(rows):
        for c in range(cols):
            y0 = r * cell_height
            x0 = c * cell_width
            y1 = y0 + cell_height
            x1 = x0 + cell_width
            cell = frame_tresh[y0:y1, x0:x1]
            proportion = float(np.mean(cell) / 255)
            grid[r, c] = 1 if proportion >= 0.5 else 0 # TODO remove magic number

    return frame, grid


def build_grid(frame: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """
    Prepares the grid and frame for visualization

    :param frame:
    :param grid:

    :returns: The Numpy array grid to display
    """
    rows, cols = grid.shape
    h, w = frame.shape[:2]
    vis = np.ones_like(frame, dtype=np.uint8) * 255  # fond blanc

    cell_height = h // rows
    cell_width = w // cols

    # lignes de grille
    for r in range(1, rows):
        y = r * cell_height
        cv2.line(vis, (0, y), (w, y), (128, 128, 128), 5)
    for c in range(1, cols):
        x = c * cell_width
        cv2.line(vis, (x, 0), (x, h), (128, 128, 128), 5)

    return vis


def stop_camera(cap: cv2.VideoCapture):
    """
    Stops the video capture for a given capture object.

    :param cap: The video capture object.
    """
    cap.release()


if __name__ == "__main__":
    print('Cameras found', list_cameras())
