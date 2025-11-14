from src.utils import *
from src.vision import *

# Initialization
cap = start_vision()

# Main loop
while True:
    # Stop command
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting with command 'q'")
        break

    # Capture and compute vision data
    frame, grid, projected, robot = get_vision_data(cap)

    # ———————————————————————
    # HERE MAIN LOGIC (LOCAL NAV, GLOBAL NAV, FILTERING, ETC.)
    # ———————————————————————

    # Visualization
    vis = build_grid(frame, grid)
    combined = np.hstack((frame, vis))
    draw_control_room(combined)

# Stop
stop_vision(cap)
