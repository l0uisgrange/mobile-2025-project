from time import sleep

import asyncio
from tdmclient import ClientAsync

from src.navigation import Navigation
from src.motion import Motion
from src.kalman import ThymioEKF
from src.vision import *

async def main():
    with ClientAsync() as client:
        with await client.lock() as node:
            await node.wait_for_variables({"motor.left.target", "motor.right.target", "prox.horizontal", "motor.left.speed", "motor.right.speed"})
            mot = Motion(node)
            vis = Vision()
            nav = Navigation()
            kalman = None
            

            # Wait a second for camera connection time
            sleep(1)

            # TODO: maybe remove
            path = []
            plan = []

            # Main loop
            while True:
                # ——— VISION (src.vision) ————————————————————————————

                # Step function
                vis.step()

                # ————— Kalman Filter —————

                if vis.lock and kalman is None:
                    kalman = ThymioEKF([vis.robot[1][1], vis.robot[1][0], vis.robot[0]], dt=DT)
                
                if kalman is not None:
                    vL = node.v.motor.left.speed
                    vR = node.v.motor.right.speed
                    kalman.step(vL, vR, [vis.robot[1][1], vis.robot[1][0], vis.robot[0]] if vis.get_trust() else None)

                print("Vision pose: " + "(x: " + str(vis.robot[1][1]) + ", y: " + str(vis.robot[1][0]) + ", theta: " + str(vis.robot[0]) + ")" ) if vis.robot is not None else print("Vision pose: N/A")
                print("Encoder only pose: " + "(x: " + str(kalman.w[0]) + ", y: " + str(kalman.w[1]) + ", theta: " + str(kalman.w[2]) + ")" ) if kalman is not None else print("Encoder only pose: N/A")
                print("Kalman pose: " + "(x: " + str(kalman.x[0]) + ", y: " + str(kalman.x[1]) + ", theta: " + str(kalman.x[2]) + ")" ) if kalman is not None else print("Kalman pose: N/A")

                # ——————————

                # ——— NAVIGATION (src.navigation) ————————————————————

                # TODO: chnage here depending on Kalman. Retirer le and not lock aussi.
                if vis.get_trust() and vis.lock:
                    if not plan:
                        (plan, path) = nav.update_plan(
                            vis.grid, vis.robot[1], vis.target)

                # ——— MOTION (src.motion) ————————————————————————————

                if plan and vis.get_trust():  # TODO: retirer get_trust/changer ca avec filtre de kalman etc
                    await mot.follow_path(plan, (vis.robot[1][0], vis.robot[1][1], vis.robot[0]))
                    await client.sleep(DT)

                # ——— VISUALIZATION (src.utils) ——————————————————————

                view = vis.render_grid(path, plan)
                if view is None:
                    continue
                blended = cv2.addWeighted(
                    vis.get_frame(), FRAME_OPACITY, view.astype(np.uint8), GRID_OPACITY, 0)
                draw_control_room(vis, blended)

    # Stop
    vis.release()


asyncio.run(main())
