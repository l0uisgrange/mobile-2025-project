from src.utils import show_grid
from src.vision import *
from src.vision import build_grid

## Initialization
cap = start_camera()

## Main loop
while True:
    # Stop command
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting with command 'q'")
        break
    frame, grid = get_grid(cap)
    vis = build_grid(frame, grid)
    show_grid(frame, vis)

## Stop
stop_camera(cap)
