from time import sleep
import math

#motion
import asyncio
from tdmclient import ClientAsync

from src.navigation import Navigation
from src.motion import Motion
from src.vision import *

async def main():
    # Initialization
    #motion = None
    with ClientAsync() as client:
        with await client.lock() as node:
            await node.wait_for_variables({"motor.left.target", "motor.right.target", "prox.horizontal"})
            motion = Motion(node)
            vis = Vision()
            nav = Navigation()

            # Wait a second for camera connection time
            sleep(2)

            # # TODO remove when getter inside navigation is ready
            path = []
            plan = []
            #TODO:DEBUG
            toogle = False

            # Main loop
            while True:
                # Stop command
                key = cv2.waitKey(2) & 0xFF
                # Exit main loop
                if key == ord('q'):
                    break

                #DEBUG
                if key == ord('s'):
                    await motion.toggle()
                    toogle = not toogle


                # ——————————————————————————————————————————————
                # VISION (src.vision)
                # ——————————————————————————————————————————————

                vis.step()

                # ——————————————————————————————————————————————
                # Navigation
                # ——————————————————————————————————————————————


                if vis.get_trust():
                    #plan = nav.update_plan(vis.grid, plan, vis.robot[1], vis.target) # Filtre modifier
                    #print(plan)
                    #nav.grid = copy.deepcopy(vis.grid)
                #     print(vis.grid)
                    if not plan:
                        plan = nav.update_plan(vis.grid, plan, vis.robot[1], vis.target)[1]



                #     nav.grid = copy.deepcopy(vis.grid)
                    
                #     for r in range(nav.grid.shape[0]):
                #         for c in range(nav.grid.shape[1]):
                #             if nav.grid[r, c] == CELL_OBSTACLE or vis.grid[r, c] == CELL_MARGIN:
                #                 nav.grid[r, c] = CELL_OBSTACLE
                #             else:
                #                 nav.grid[r, c] = CELL_VOID

                #     path = nav.a_star(vis.robot[1], vis.target)
                # #     print(path)
                #     if path is not None:
                #         #pass
                #         plan = nav.generate_plan(vis.robot[1], vis.target)





                    
                #     print(path)
                #     print(plan)

                # ——————————————————————————————————————————————
                # Motion
                # ——————————————————————————————————————————————
                    if plan and not toogle:
                        await motion.follow_path(plan, (vis.robot[1][0],vis.robot[1][1], vis.robot[0]))
                        await client.sleep(DT)

                # ——————————————————————————————————————————————
                # VISUALIZATION
                # ——————————————————————————————————————————————

    view = vis.render_grid(path, plan)
    if view is None:
        continue
    blended = cv2.addWeighted(vis.get_frame(), FRAME_OPACITY, view.astype(np.uint8), GRID_OPACITY, 0)
    draw_control_room(vis, blended)

            # Stop
            vis.release()


asyncio.run(main())