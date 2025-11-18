import cv2
import numpy as np
from numpy import ndarray

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


def start_vision(index: int = 0) -> cv2.VideoCapture:
    """
    Starts the video capture and creates a window for a given camera index.

    :returns: The video capture object.
    """
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setWindowTitle(WINDOW_NAME, "Control Center")
    cap = cv2.VideoCapture(index)
    return cap


def get_grid(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    """
    Reads a frame from the given video capture and returns the grid.

    :param frame: Frame

    :returns: Scene frame, grid and frame_tresh with obstacles
    """

    # Filtering
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_filtered = cv2.bilateralFilter(frame_gray,
                                         VISION_FILTER_DIAM,
                                         VISION_FILTER_SIGMA_COLOR,
                                         VISION_FILTER_SIGMA_SPACE)

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

    return frame, grid


def build_grid(frame: np.ndarray, grid: np.ndarray, robot: tuple[float, tuple[float, float]] | None) -> np.ndarray:
    """
    Prepares the grid and frame for visualization

    :param robot: Current robot position and orientation
    :param frame: Frame
    :param grid: Grid data

    :returns: The Numpy array grid to display
    """
    rows, cols = grid.shape
    h, w = frame.shape[:2]
    vis = np.ones_like(frame, dtype=np.uint8) * 255

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

    # Gris lines
    for r in range(1, rows):
        y = r * cell_height
        cv2.line(vis, (0, y), (w, y), GRID_COLOR, GRID_THICKNESS)
    for c in range(1, cols):
        x = c * cell_width
        cv2.line(vis, (x, 0), (x, h), GRID_COLOR, GRID_THICKNESS)

    # Robot position
    if robot is not None:
        orientation, (x, y) = robot
        cv2.rectangle(vis, (int(x), int(y)), (int(x + 20), int(y + 20)), COLOR_RED, thickness=-1)

    return vis


def stop_vision(cap: cv2.VideoCapture):
    """
    Stops the video capture for a given capture object.

    :param cap: The video capture object.
    """
    cv2.destroyAllWindows()
    cap.release()


def get_markers(frame: np.ndarray):
    """
    Get aruko markers information from given frame.

    :param frame: The frame.
    """
    # Detect markers using aruco OpenCV module
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(frame)

    return corners, ids, rejected


def aruko_projection(frame: np.ndarray, markers) -> tuple[np.ndarray, bool]:
    """
    Stops the video capture for a given capture object.

    :param frame: The frame.
    """
    # Extract markers data
    corners, ids, rejected = markers

    # Detect markers using aruco OpenCV module
    height, width = frame.shape[:2]

    # Projection
    projection_points = np.float32([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1]
    ])

    if ids is not None and len(ids) >= 4:
        marker_centers = {}
        # Find aruco symbol center
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in VISION_MARKERS:
                continue
            c = corners[i][0]
            center_x = np.mean(c[:, 0])
            center_y = np.mean(c[:, 1])
            marker_centers[marker_id] = [center_x, center_y]

        pts_src = np.float32([marker_centers[mid] for mid in VISION_MARKERS if mid in marker_centers])

        # Project onto aruco
        if len(projection_points) != len(pts_src):
            print("Not enough markers detected")
            return frame, False
        M = cv2.getPerspectiveTransform(pts_src, projection_points)
        projected = cv2.warpPerspective(frame, M, (width, height))
        return projected, True

    return frame, False


def get_robot(frame: np.ndarray, markers) -> tuple[float, tuple[float, float]] | None:
    """
    Computes the position and orientation of the robot from the video frame.

    :param frame: The video frame.
    :returns: The robot's position and orientation.
    """
    # Extract markers data
    corners, ids, rejected = markers

    if ids is not None and VISION_ROBOT_MARKER in ids:
        index = np.where(ids == VISION_ROBOT_MARKER)[0][0]
        center = np.mean(corners[0][index], axis=0)
        orientation = 0
        return orientation, center

    return None


def get_vision_data(cap: cv2.VideoCapture):
    """
    Captures the vision grid from the video capture object, computes the position of the robot and uses ARUCO markers.

    :param cap: The video capture object.

    :returns: The frame, grid, projected state and robot position and orientation
    """
    frame = get_image(cap)
    markers = get_markers(frame)
    frame, projected = aruko_projection(frame, markers)
    frame, grid = get_grid(frame)
    robot = get_robot(frame, markers)

    return frame, grid, projected, robot


def get_image(cap: cv2.VideoCapture) -> ndarray | None:
    ret, frame = cap.read()
    if not ret:
        return None
    return frame


if __name__ == "__main__":
    print('Cameras found', list_cameras())
