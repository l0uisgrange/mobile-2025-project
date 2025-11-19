from src.utils import *


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


def set_targets_grid(frame: np.ndarray, robot: tuple[float, tuple[int, int]], end: tuple[int, int]) -> np.ndarray | None:
    """
    Reads a frame from the given video capture and returns the grid.

    :param frame: Frame

    :returns: Scene frame, grid and frame_tresh with obstacles and targets
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
            grid[r, c] = CELL_OBSTACLE if np.mean(cell) > 255 / 2 else CELL_VOID

    if robot is not None:
        grid[robot[1][0], robot[1][1]] = CELL_ROBOT
    if end is not None:
        grid[end[0], end[1]] = CELL_TARGET

    return grid


def render_grid(frame: np.ndarray, grid: np.ndarray, robot: tuple[float, tuple[float, float]] | None) -> np.ndarray:
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
            color = COLOR_BLACK
            if grid[r, c] == CELL_ROBOT:
                color = COLOR_RED
            elif grid[r, c] == CELL_UPSCALE:
                color = COLOR_GRAY
            elif grid[r, c] == CELL_TARGET:
                color = COLOR_GREEN
            if grid[r, c] != CELL_VOID:
                cv2.rectangle(vis, (x0, y0), (x1 - 1, y1 - 1), color, thickness=-1)

    # Gris lines
    for r in range(1, rows):
        y = r * cell_height
        cv2.line(vis, (0, y), (w, y), GRID_COLOR, GRID_THICKNESS)
    for c in range(1, cols):
        x = c * cell_width
        cv2.line(vis, (x, 0), (x, h), GRID_COLOR, GRID_THICKNESS)

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


def aruko_projection(frame: np.ndarray, markers) -> tuple[np.ndarray, bool, np.ndarray | None]:
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
            return frame, False, None
        M = cv2.getPerspectiveTransform(pts_src, projection_points)
        projected = cv2.warpPerspective(frame, M, (width, height))
        return projected, True, M

    return frame, False, None


def get_targets(frame: np.ndarray, markers, matrix: np.ndarray | None) -> tuple[tuple[float, tuple[int, int]] | None, tuple[int, int] | None]:
    """
    Computes the position and orientation of the robot and the position of the target from the video frame.

    :param matrix: Projection matrix
    :param frame: The current image frame
    :param markers: The markers' data.
    :returns: The robot's position and orientation and the target's position.
    """
    # Extract markers data
    corners, ids, rejected = markers

    if matrix is None:
        return None, None

    robot = None
    orientation = 0
    end = None

    # Find robot
    if ids is not None and VISION_ROBOT_MARKER in ids:
        index = np.where(ids == VISION_ROBOT_MARKER)[0][0]
        center = np.mean(corners[index][0], axis=0)
        robot = orientation, to_grid_units(frame, project_point(center, matrix))

    # Find the end target
    if ids is not None and VISION_TARGET_MARKER in ids:
        index = np.where(ids == VISION_TARGET_MARKER)[0][0]
        center = np.mean(corners[index][0], axis=0)
        end = to_grid_units(frame, project_point(center, matrix))

    return robot, end


def get_image(cap: cv2.VideoCapture) -> np.ndarray | None:
    ret, frame = cap.read()
    if not ret:
        return None
    return frame

def project_point(point: tuple[float, float], matrix: np.ndarray) -> tuple[float, float]:
    """
    Projects a point in the original frame to the grid

    :param point: The point in the original frame
    :param matrix: The projection matrix
    :returns point: The coordinate on the grid plane
    """
    pt = np.array([[[float(point[0]), float(point[1])]]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(pt, matrix)
    return float(transformed[0][0][0]), float(transformed[0][0][1])


if __name__ == "__main__":
    print('Cameras found', list_cameras())