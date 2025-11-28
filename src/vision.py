import copy
from typing import Any

import cv2
import numpy as np

from src.utils import *
import math


class Vision:
    cap: cv2.VideoCapture
    grid: np.ndarray | None = None
    trust: bool = False
    lock: bool = False
    raw_frame: np.ndarray | None = None
    per_frame: np.ndarray | None = None
    markers: tuple[Any, Any, Any] | None = None
    matrix: np.ndarray | None = None
    robot: tuple[float, tuple[int, int]] | None = None
    target: tuple[int, int] | None = None

    def __init__(self, camera=0):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setWindowTitle(WINDOW_NAME, "Control Center")
        self.cap = cv2.VideoCapture(camera)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

    def step(self):
        """
        Whole vision analysis
        """
        # Resize window to ensure correct sizing
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

        # 1. Capture frame
        self.capture()
        if self.raw_frame is None:
            self.trust = False
            return
        # 2. Detect aruco markers
        self.detect_markers()
        if not self.lock and (self.markers is None or self.markers[1] is None or not all(
                mid in self.markers[1].flatten() for mid in VISION_MARKERS)):
            self.trust = False
            return
        self.aruco_projection()
        if self.matrix is None:
            self.trust = False
            return
        # 3. Find targets
        self.find_targets()
        if self.target is None or self.robot is None:
            self.trust = False
            return
        # 4. Build final grid
        if not self.lock:
            self.build_grid()
        self.trust = True

    def capture(self):
        """
        Captures a frame from the camera.
        """
        ret, frame = self.cap.read()
        self.raw_frame = frame if ret else None

    def detect_markers(self):
        """
        Compute markers' position and orientation from the current frame.
        """
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        self.markers = detector.detectMarkers(self.raw_frame)

    def aruco_projection(self):
        """
        Creates a projected frame using aruco markers.
        ┌───┬───┐
        │ 1 │ 2 │
        ├───┼───┤ The frame visible on the screen.
        │ 4 │ 3 │
        └───┴───┘
        """
        height, width = self.raw_frame.shape[:2]
        frame = self._hide_markers(VISION_ALL_MARKERS)

        if self.lock:
            self.per_frame = cv2.warpPerspective(frame, self.matrix, (width, height))
            return

        corners, ids, rejected = self.markers
        indices = {marker_id[0]: i for i, marker_id in enumerate(ids)}

        projection_points = np.float32([
            [0, 0],
            [width - 1, 0],
            [width - 1, height - 1],
            [0, height - 1]
        ])

        markers_centers = []
        for marker_id in VISION_MARKERS:
            index = indices[marker_id]
            c = corners[index][0]
            center_x = np.mean(c[:, 0])
            center_y = np.mean(c[:, 1])
            markers_centers.append([center_x, center_y])

        markers_centers = np.float32(markers_centers)

        # Project onto aruco
        self.matrix = cv2.getPerspectiveTransform(markers_centers, projection_points)
        self.per_frame = cv2.warpPerspective(frame, self.matrix, (width, height))

    def build_grid(self):
        """
        Builds grid from the current projected frame.
        """
        frame_gray = cv2.cvtColor(self.per_frame, cv2.COLOR_BGR2GRAY)
        frame_filtered = cv2.bilateralFilter(frame_gray, VISION_FILTER_DIAM, VISION_FILTER_SIGMA_COLOR,
                                             VISION_FILTER_SIGMA_SPACE)
        _, frame_tresh = cv2.threshold(frame_filtered, VISION_TRESH, VISION_TRESH_MAX, cv2.THRESH_BINARY_INV)
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

        # Margin in obstacles
        obstacle_mask = (grid == CELL_OBSTACLE).astype(np.uint8) * 255
        ksize = max(1, 2 * VISION_OBSTACLE_MARGIN + 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        dilated = cv2.dilate(obstacle_mask, kernel)
        margin_positions = (dilated > 0) & (grid == CELL_VOID)
        grid[margin_positions] = CELL_MARGIN

        self.grid = grid

    def _hide_markers(self, markers):
        """
        Adds white rectangle to hide markers (otherwise they're considered obstacles)
        """
        corners, ids, rejected = self.markers
        indices = {marker_id[0]: i for i, marker_id in enumerate(ids)}
        frame = self.raw_frame
        for marker_id in markers:
            if marker_id not in indices:
                continue
            index = indices[marker_id]
            pts = self.markers[0][index][0].reshape(-1, 1, 2)
            rect = cv2.minAreaRect(pts)
            (cx, cy), (w_box, h_box), angle = rect
            w_box = max(1.0, w_box + 2 * VISION_MARKERS_PADDING)
            h_box = max(1.0, h_box + 2 * VISION_MARKERS_PADDING)
            box = cv2.boxPoints(((cx, cy), (w_box, h_box), angle))
            box = np.int32(np.round(box))
            cv2.fillPoly(frame, [box], COLOR_WHITE)
        return frame

    def render_grid(self, path, plan):
        """
        Creates a 3D BGR np.ndarray grid from the current grid data.
        """
        if not self.trust:
            return np.ones_like(self.raw_frame, dtype=np.uint8) * 255

        h, w = self.per_frame.shape[:2]
        vis = np.ones_like(self.per_frame, dtype=np.uint8) * 255

        rows, cols = GRID_SHAPE

        cell_height = h // rows
        cell_width = w // cols

        path_set = set(path) if path is not None else set()
        plan_set = set(plan) if plan is not None else set()

        grid = copy.deepcopy(self.grid)

        # Add robot and target
        if self.robot is not None:
            grid[self.robot[1][0], self.robot[1][1]] = CELL_ROBOT
        if self.target is not None:
            grid[self.target[0], self.target[1]] = CELL_TARGET
        # Place robot and target
        if abs(self.robot[1][0]) > GRID_SHAPE[0] or abs(self.robot[1][1]) > GRID_SHAPE[1]:
            self.robot = None
        if abs(self.target[0]) > GRID_SHAPE[0] or abs(self.target[1]) > GRID_SHAPE[1]:
            self.target = None

        for r in range(rows):
            for c in range(cols):
                y0 = r * cell_height
                x0 = c * cell_width
                y1 = h if r == rows - 1 else (y0 + cell_height)
                x1 = w if c == cols - 1 else (x0 + cell_width)
                color = COLOR_BLACK
                if grid[r, c] == CELL_MARGIN:
                    color = COLOR_GRAY
                if (r, c) in plan_set:
                    color = COLOR_BLUE
                elif (r, c) in path_set:
                    color = COLOR_LIGHTRED
                if grid[r, c] == CELL_ROBOT:
                    color = COLOR_RED
                elif grid[r, c] == CELL_TARGET:
                    color = COLOR_GREEN
                if grid[r, c] != CELL_VOID or (r, c) in path_set or (r, c) in plan_set:
                    cv2.rectangle(vis, (x0, y0), (x1 - 1, y1 - 1), color, thickness=-1)

        # Grid lines
        for r in range(1, rows):
            y = r * cell_height
            cv2.line(vis, (0, y), (w, y), GRID_COLOR, GRID_THICKNESS)
        for c in range(1, cols):
            x = c * cell_width
            cv2.line(vis, (x, 0), (x, h), GRID_COLOR, GRID_THICKNESS)

        return vis

    def find_targets(self):
        """
        Uses aruko markers and computes the target.
        """
        corners, ids, rejected = self.markers

        robot = None
        target = None

        if ids is None or corners is None:
            self.robot = robot
            self.target = target
            return

        flat_ids = ids.flatten()

        # Find robot
        if VISION_ROBOT_MARKER in flat_ids:
            index = np.where(flat_ids == VISION_ROBOT_MARKER)[0][0]
            center = np.mean(corners[index][0], axis=0)
            orientation = self._angle_projection(VISION_ROBOT_MARKER)
            robot = orientation, to_grid_units(self.per_frame, self._point_projection(center))

        # Find the target
        if VISION_TARGET_MARKER in flat_ids:
            index = np.where(flat_ids == VISION_TARGET_MARKER)[0][0]
            center = np.mean(corners[index][0], axis=0)
            target = to_grid_units(self.per_frame, self._point_projection(center))

        self.robot = robot
        self.target = target

    def release(self):
        """
        Destroys windows and releases the capture object.
        """
        cv2.destroyAllWindows()
        self.cap.release()

    def _point_projection(self, point: tuple[float, float] | Any):
        """
        Projects a point using the aruco matrix.
        """
        re_point = point.astype(np.float32).reshape(1, 1, 2)
        return cv2.perspectiveTransform(re_point, self.matrix)[0][0]

    def _angle_projection(self, id: int):
        """
        Projects a marker angle.
        """
        corners, ids, _ = self.markers
        if ids is None:
            return None
        flat_ids = ids.flatten()
        matches = np.where(flat_ids == id)[0]
        if matches.size == 0:
            return None
        idx = matches[0]
        pts = corners[idx][0].astype(np.float32)
        pts = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), self.matrix).reshape(-1, 2)

        # Use cv2 to get the marker angle
        # _, _, angle = cv2.minAreaRect(pts)
        # angle = (angle + 360.0) % 360.0

        p0 = pts[0]
        p1 = pts[1]
        #print(p0)
        #print(p1)
        angle = -math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        #print(angle)
        return angle

    def get_frame(self):
        """
        Simple frame getter.
        """
        if self.trust:
            return self.per_frame
        else:
            return self.raw_frame

    def get_grid(self):
        """
        Simple grid getter.
        """
        return self.grid

    def get_trust(self):
        """
        An external used method to check if the data provided by vision are trustworthy.
        """
        return self.robot is not None and self.target is not None and self.trust

    def set_lock(self, lock: bool):
        """
        Locks the corner markers to prevent blinking (catch and no catch)
        """
        self.lock = lock
