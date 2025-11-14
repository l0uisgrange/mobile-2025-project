import cv2
import numpy as np

from src.consts import *


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


def get_grid(cap: cv2.VideoCapture) -> tuple[np.ndarray, np.ndarray] | None:
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
    frame_filtered = cv2.bilateralFilter(frame_gray, VISION_FILTER_DIAM, VISION_FILTER_SIGMA_COLOR, VISION_FILTER_SIGMA_SPACE)

    # Treshhold
    _, frame_tresh = cv2.threshold(frame_filtered, VISION_TRESH, VISION_TRESH_MAX, cv2.THRESH_BINARY_INV)

    # Build grid
    rows, cols = GRID_SHAPE
    h, w = frame_tresh.shape
    cell_height = h // rows
    cell_width = w // cols
    grid = np.zeros((rows, cols), dtype=int)
    for r in range(rows):
        for c in range(cols):
            y0 = r * cell_height
            x0 = c * cell_width
            y1 = h if r == rows - 1 else (y0 + cell_height)
            x1 = w if c == cols - 1 else (x0 + cell_width)
            cell = frame_tresh[y0:y1, x0:x1]
            proportion = float(np.mean(cell) / 255)
            grid[r, c] = round(proportion)

    return frame_filtered, grid


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

    for r in range(rows):
        for c in range(cols):
            y0 = r * cell_height
            x0 = c * cell_width
            y1 = h if r == rows - 1 else (y0 + cell_height)
            x1 = w if c == cols - 1 else (x0 + cell_width)
            if grid[r, c]:
                cv2.rectangle(vis, (x0, y0), (x1 - 1, y1 - 1), COLOR_BLACK, thickness=-1)

    # lignes de grille
    for r in range(1, rows):
        y = r * cell_height
        cv2.line(vis, (0, y), (w, y), GRID_COLOR, GRID_THICKNESS)
    for c in range(1, cols):
        x = c * cell_width
        cv2.line(vis, (x, 0), (x, h), GRID_COLOR, GRID_THICKNESS)

    return vis


def stop_camera(cap: cv2.VideoCapture):
    """
    Stops the video capture for a given capture object.

    :param cap: The video capture object.
    """
    cap.release()


if __name__ == "__main__":
    print('Cameras found', list_cameras())
