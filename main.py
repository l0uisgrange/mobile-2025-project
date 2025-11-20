from src.navigation import Navigation
from src.vision import *

# Initialization
vis = Vision()
nav = Navigation()

# Main loop
while True:
    # Stop command
    if cv2.waitKey(2) & 0xFF == ord('q'):
        print("Exiting with command 'q'")
        break

    # ——————————————————————————————————————————————
    # VISION (src.vision)
    # ——————————————————————————————————————————————

    vis.capture()
    vis.detect_markers()
    vis.aruco_projection()


    # Get targets' position and orientation
    vis.find_targets()
    # Create the grid with targets
    vis.build_grid()

    # ——————————————————————————————————————————————
    # LOCAL OBSTACLE AVOIDANCE (src.local)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # GLOBAL NAVIGATION (src.navigation)
    # ——————————————————————————————————————————————

    # Path finding
    #if robot is not None and end is not None:
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

    vis = vis.render_grid()
    combined_bottom = np.hstack((vis.per_frame, vis))
    #combined_top = np.hstack((frame, vis))
    #combined = np.vstack((combined_top, combined_bottom))
    alpha = 0.5
    blended = cv2.addWeighted(vis.per_frame, alpha,
                              vis.astype(np.uint8), 1.0 - alpha, 0)
    draw_control_room(blended, True, vis.robot, vis.end)

# Stop
vis.release()
