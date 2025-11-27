from time import sleep

import asyncio
from tdmclient import ClientAsync

from src.navigation import Navigation
from src.motion import Motion
from src.vision import *


async def main():
    with ClientAsync() as client:
        with await client.lock() as node:
            await node.wait_for_variables({"motor.left.target", "motor.right.target", "prox.horizontal"})
            motion = Motion(node)
            vis = Vision()
            nav = Navigation()

            # Wait a second for camera connection time
            sleep(1)

            # TODO: remove (vision locking)
            lock = False

            # TODO: maybe remove
            path = []
            plan = []
            # Main loop
            while True:
                # Stop command
                key = cv2.waitKey(2) & 0xFF
                # Exit main loop
                if key == ord('q'):
                    break
                if key == ord('l'):
                    lock = not lock

                # ——————————————————————————————————————————————
                # VISION (src.vision)
                # ——————————————————————————————————————————————

                vis.step()

                # ——————————————————————————————————————————————
                # Navigation
                # ——————————————————————————————————————————————

                # TODO: chnage here depending on Kalman. Retirer le and not lock aussi.
                if vis.get_trust() and lock:
                    if not plan:
                        (plan, path) = nav.update_plan(
                            vis.grid, vis.robot[1], vis.target)

                # ——————————————————————————————————————————————
                # Motion
                # ——————————————————————————————————————————————

                if plan:
                    await motion.follow_path(plan, (vis.robot[1][0], vis.robot[1][1], vis.robot[0]))
                    await client.sleep(DT)

                # ——————————————————————————————————————————————
                # VISUALIZATION
                # ——————————————————————————————————————————————

                view = vis.render_grid(path, plan)
                if view is None:
                    continue
                blended = cv2.addWeighted(
                    vis.get_frame(), FRAME_OPACITY, view.astype(np.uint8), GRID_OPACITY, 0)
                draw_control_room(vis, blended)

    # Stop
    vis.release()


asyncio.run(main())
