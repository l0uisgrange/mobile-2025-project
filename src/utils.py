import cv2
import numpy as np

from src.consts import *


def draw_control_room(vis, frame: np.ndarray):
    """
    Shows the grid inside a new window
    """
    # Ensure frame is 3D (BGR)
    if frame.ndim == 2:
        print("Input has wrong dimension")
        exit(1)

    # Add a status bar
    height, width, = frame.shape[:2]
    bar = np.zeros((STATUS_BAR_HEIGHT, width, 3), dtype=np.uint8)
    bar.fill(0)

    text_x = TEXT_PADDING + STATUS_INDICATOR_SIZE + STATUS_BAR_LABEL_GAP

    # Add marker status
    def show_status(name: str, position: tuple[int, int], value: bool, gray_text: str | None = None):
        marker_color = COLOR_GREEN if value else COLOR_RED
        padding = (position[0] * STATUS_BAR_SPACING[0], position[1] * STATUS_BAR_SPACING[1])
        cv2.rectangle(bar,
                      (TEXT_PADDING + padding[0], TEXT_PADDING + padding[1]),
                      (STATUS_INDICATOR_SIZE + TEXT_PADDING + padding[0],
                       STATUS_INDICATOR_SIZE + TEXT_PADDING + padding[1]),
                      marker_color, -1)
        cv2.putText(
            bar,
            name,
            (text_x + padding[0], TEXT_PADDING + STATUS_INDICATOR_SIZE - TEXT_DELTA + padding[1]),
            cv2.FONT_HERSHEY_DUPLEX,
            STATUS_BAR_FONTSCALE,
            COLOR_WHITE,
            STATUS_BAR_FONTTHICKNESS,
            cv2.LINE_AA
        )
        if gray_text is not None:
            cv2.putText(
                bar,
                gray_text,
                (text_x + padding[0] + TEXT_GRAY_POSITION,
                 TEXT_PADDING + STATUS_INDICATOR_SIZE - TEXT_DELTA + padding[1]),
                cv2.FONT_HERSHEY_DUPLEX,
                STATUS_BAR_FONTSCALE,
                COLOR_GRAY,
                STATUS_BAR_FONTTHICKNESS,
            )

    show_status("TRUST", (0, 0), vis.trust)
    show_status("ROBOT", (0, 1), vis.robot is not None,
                gray_text='(' + str(vis.robot[1][0]) + ', ' + str(vis.robot[1][1]) + ') AT ' + str(
                    int(vis.robot[0])) + ' DEGREES' if vis.robot else None)
    show_status("TARGET", (1, 0), vis.target is not None,
                gray_text='(' + str(vis.target[0]) + ', ' + str(vis.target[1]) + ')' if vis.target else None)
    show_status("PATH", (1, 1), False)

    combined_frame = np.vstack((frame, bar))

    cv2.imshow(WINDOW_NAME, combined_frame)


def to_grid_units(frame: np.ndarray, coordinate: tuple[float, float]) -> tuple[int, int]:
    """
    Converts a frame coordinate to a position in the grid
    """
    h, w = frame.shape[:2]
    x, y = coordinate
    cell_height = h // GRID_SHAPE[0]
    cell_width = w // GRID_SHAPE[1]
    return tuple((int(y // cell_height), int(x // cell_width)))
