import cv2
import numpy as np

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

    :return: The video capture object.
    """
    cap = cv2.VideoCapture(index)
    return cap


def get_frame(cap: cv2.VideoCapture) -> np.ndarray | None:
    """
    Reads a frame from the given video capture and displays it in a window.

    :param cap: Video capture object.
    """
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.bilateralFilter(gray, 15, 7, 8)

        cv2.imshow("Image", blurred)
        return frame
    else:
        print("Could not read image")
        return None

def get_grid(frame: np.ndarray) -> np.ndarray:
    """
    Reads the given frame to extract

    :param frame: Extract the grid from the frame
    """


def stop_camera(cap: cv2.VideoCapture):
    """
    Stops the video capture for a given capture object.
    """
    cap.release()
