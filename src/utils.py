import cv2
import numpy as np

from src.consts import *


def draw_control_room(frame: np.ndarray, projected: bool, robot: tuple[float, tuple[float, float]] | None):
    """
    Shows the grid inside a new window

    :param projected: Projection state
    :param robot: Robot data
    :param frame: The raw image frame
    """
    # Ensure frame is 3D (BGR)
    if frame.ndim == 2:
        print("Input has wrong dimension")
        exit(1)

    # Add status bar
    height, width, = frame.shape[:2]
    bar = np.zeros((STATUS_BAR_HEIGHT, width, 3), dtype=np.uint8)
    bar.fill(0)

    text_x = TEXT_PADDING + STATUS_INDICATOR_SIZE + STATUS_BAR_LABEL_GAP

    # Add marker status
    marker_color = COLOR_GREEN if projected else COLOR_RED
    cv2.rectangle(bar,
                  (TEXT_PADDING, TEXT_PADDING),
                  (STATUS_INDICATOR_SIZE + TEXT_PADDING, STATUS_INDICATOR_SIZE + TEXT_PADDING),
                  marker_color, -1)
    cv2.putText(
        bar,
        "MARKERS",
        (text_x, TEXT_PADDING + STATUS_INDICATOR_SIZE - TEXT_DELTA),
        cv2.FONT_HERSHEY_DUPLEX,
        1,
        COLOR_WHITE,
        2,
        cv2.LINE_AA
    )

    # Add robot status
    marker_color = COLOR_RED if robot is None else COLOR_GREEN
    cv2.rectangle(bar,
                  (TEXT_PADDING, TEXT_PADDING + STATUS_BAR_SPACING),
                  (STATUS_INDICATOR_SIZE + TEXT_PADDING, STATUS_INDICATOR_SIZE + TEXT_PADDING + STATUS_BAR_SPACING),
                  marker_color, -1)
    cv2.putText(
        bar,
        "ROBOT",
        (text_x, TEXT_PADDING + STATUS_INDICATOR_SIZE + STATUS_BAR_SPACING - TEXT_DELTA),
        cv2.FONT_HERSHEY_DUPLEX,
        1,
        COLOR_WHITE,
        2,
        cv2.LINE_AA
    )
    if robot is not None:
        orientation, (x, y) = robot
        cv2.putText(
            bar,
            '(' + str(int(x)) + ', ' + str(int(y)) + ') AT ' + str(int(orientation)) + ' DEGREES',
            (text_x + TEXT_ROBOT_POSITION, TEXT_PADDING + STATUS_INDICATOR_SIZE + STATUS_BAR_SPACING - TEXT_DELTA),
            cv2.FONT_HERSHEY_DUPLEX,
            1,
            COLOR_GRAY,
            2,
            cv2.LINE_AA
        )

    combined_frame = np.vstack((frame, bar))

    cv2.imshow(WINDOW_NAME, combined_frame)


def to_grid_units(frame: np.ndarray, coordinate: tuple[float, float]) -> tuple[int, int]:
    """
    Converts a frame coordinate to a position in the grid

    :param frame: The image frame
    :param coordinate: The coordinate on the frame
    :returns position: The position in the grid
    """
    h, w = frame.shape[:2]
    x, y = coordinate
    cell_height = h // GRID_SHAPE[0]
    cell_width = w // GRID_SHAPE[1]
    return tuple((int(y // cell_height), int(x // cell_width)))
