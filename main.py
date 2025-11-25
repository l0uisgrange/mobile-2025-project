from time import sleep

from src.navigation import Navigation
from src.vision import *

# Initialization
vis = Vision()
nav = Navigation()

# Wait a second for camera connection time
sleep(1.5)

# TODO remove when getter inside navigation is ready
path = []
plan = []

# Main loop
while True:
    # Stop command
    key = cv2.waitKey(2) & 0xFF
    # Exit main loop
    if key == ord('q'):
        break

    # ——————————————————————————————————————————————
    # VISION (src.vision)
    # ——————————————————————————————————————————————

    vis.step()

    # ——————————————————————————————————————————————
    # LOCAL OBSTACLE AVOIDANCE (src.local)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # GLOBAL NAVIGATION (src.navigation)
    # ——————————————————————————————————————————————

    # Path finding
    if vis.get_trust():
        path = nav.a_star(vis.robot[1], vis.target)
        if path is not None:
            plan = nav.generate_plan(vis.robot[1], vis.target)

    # ——————————————————————————————————————————————
    # MOTION (src.motion)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # VISUALIZATION
    # ——————————————————————————————————————————————

    view = vis.render_grid(path, plan)
    blended = cv2.addWeighted(vis.get_frame(), FRAME_OPACITY, view.astype(np.uint8), GRID_OPACITY, 0)
    draw_control_room(vis, blended)

# Stop
vis.release()
