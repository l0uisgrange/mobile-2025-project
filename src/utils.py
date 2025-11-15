import cv2
import numpy as np

from src.consts import *


def draw_control_room(frame: np.ndarray, projected: bool, robot: tuple[float, float]):
    """
    Shows the grid inside a new window

    :param frame: The raw image frame
    """
    # Converting from grayscale (2D) to RGB (3D)
    if frame.ndim == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

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

    # Add robot position
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
        cv2.putText(
            bar,
            '(' + ','.join(str(val) for val in robot) + ')',
            (text_x + TEXT_ROBOT_POSITION, TEXT_PADDING + STATUS_INDICATOR_SIZE + STATUS_BAR_SPACING - TEXT_DELTA),
            cv2.FONT_HERSHEY_DUPLEX,
            1,
            COLOR_GRAY,
            2,
            cv2.LINE_AA
        )

    combined_frame = np.vstack((frame, bar))

    cv2.imshow(WINDOW_NAME, combined_frame)
