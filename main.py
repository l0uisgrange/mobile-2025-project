from src.navigation import Navigation
from src.vision import *

# Initialization
cap = start_vision(0)
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

    # Capture and compute vision data
    frame = get_image(cap)
    if frame is None:
        continue
    # Catch aruko markers
    markers = get_markers(frame)
    # Project the plane accordingly
    proj_frame, projected, proj_matrix = aruko_projection(frame, markers)
    # Get targets' position and orientation
    robot, end = get_targets(proj_frame, markers, proj_matrix)
    # Create the grid with targets
    grid = set_targets_grid(proj_frame, robot, end)

    # ——————————————————————————————————————————————
    # LOCAL OBSTACLE AVOIDANCE (src.local)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # GLOBAL NAVIGATION (src.navigation)
    # ——————————————————————————————————————————————

    # Path finding
    if robot is not None and end is not None:
        nav.path = nav.a_star(robot[1], end)
        if nav.path is not None:
            nav.plan = nav.generate_plan(robot[1], end)
    # Populate the grid with path and plan
    # frame, grid = set_path_grid(grid, nav.path, nav.plan)

    # ——————————————————————————————————————————————
    # MOTION (src.motion)
    # ——————————————————————————————————————————————

    # HERE

    # ——————————————————————————————————————————————
    # VISUALIZATION
    # ——————————————————————————————————————————————

    vis = render_grid(proj_frame, grid, robot)
    combined_bottom = np.hstack((proj_frame, vis))
    #combined_top = np.hstack((frame, vis))
    #combined = np.vstack((combined_top, combined_bottom))
    alpha = 0.5
    blended = cv2.addWeighted(proj_frame, alpha,
                              vis.astype(np.uint8), 1.0 - alpha, 0)
    draw_control_room(blended, projected, robot, end)

# Stop
stop_vision(cap)
