from time import sleep

from src.navigation import Navigation
from src.vision import *

# Initialization
vis = Vision()
nav = Navigation()

# Wait a second for camera connection time
sleep(1)

# Main loop
while True:
    # Stop command
    if cv2.waitKey(2) & 0xFF == ord('q'):
        print("Exiting with command 'q'")
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
    # if robot is not None and end is not None:
    #    nav.path = nav.a_star(robot[1], end)
    #    if nav.path is not None:
    #        nav.plan = nav.generate_plan(robot[1], end)
    # Populate the grid with path and plan
    # frame, grid = set_path_grid(grid, nav.path, nav.plan)

    # ——————————————————————————————————————————————
    # MOTION (src.motion)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # VISUALIZATION
    # ——————————————————————————————————————————————

    view = vis.render_grid()
    blended = cv2.addWeighted(vis.get_frame(), GRID_OPACITY, view.astype(np.uint8), 1.0 - GRID_OPACITY, 0)
    draw_control_room(blended, vis.get_trust(), vis.robot, vis.target)

# Stop
vis.release()
