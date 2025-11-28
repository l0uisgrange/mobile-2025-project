"""
pose type expected : (x: float, y: float, theta: float)

Motion module
"""

from src.consts import *
import math


class Motion:
    def __init__(self, node):
        self.node = node

    # async def to_waypoint(self, waypoint, pose):
    #     x_wp, y_wp = waypoint[X], waypoint[Y]

    #     while True:

    #         # Calculer du robot jusqu'au waypoint
    #         dx = x_wp - pose[X]
    #         dy = y_wp - pose[Y]
    #         distance = (dx**2 + dy**2)**0.5

    #         # Check if waypoint is reached
    #         if distance < DISTANCE_TOLERANCE:

    async def follow_path(self, path, pose):
        if self.check_reached(path, pose):
            if len(path) == 0:
                return  # Path completed

            await self.follow_path(path, pose)
            return

        if self.alligned(pose, path):
            await self.goto_waypoint(path, pose)
        else:
            await self.allign(pose, path)

        #     return
        # self.allign() # Check if robot is alligned with nex waypoint, if not allign it (without moving forward). Return True if alligned withing tolerance.
        # self.controller() # Only if alligns returns True, move forward toward waypoint.

    async def goto_waypoint(self, path, pose):
        global_controller_target_speeds = self.global_controller(path, pose)
        await self.local_controller(global_controller_target_speeds)
    
    async def local_controller(self, global_controller_target_speeds):
        #Detect obstacle with prox horizontals sensors
        #Adjusts brake speeds of each wheel proportionally to the detected obstacle distance and distance of it

        #reads the proximity sensors
        # while True:
        #     #await self.node.wait_for_variables({"prox.horizontal"})
        #     prox_values = self.node.v.prox.horizontal
        #     print(f"Prox values: ",list(prox_values))
        #     await client.sleep(0.1)

        prox_values = list(self.node.v.prox.horizontal)
        #prox_values[0] is leftmost sensor, prox_values[4] is rightmost sensor
        #prox_values[1] is left-center sensor, prox_values[3] is right-center sensor
        #prox_values[2] is center sensor

        #Calculate brake speeds
        #brake_left = 0
        #brake_right = 0
        for i in range(5):
            match i:
                case 0: #leftmost sensor
                    global_controller_target_speeds[1] -= prox_values[i] * 0.07
                case 1: #left-center sensor
                    global_controller_target_speeds[1] -= prox_values[i] * 0.14
                case 2: #center sensor
                    global_controller_target_speeds[0] -= prox_values[i] * 0.28
                case 3: #right-center sensor
                    global_controller_target_speeds[0] -= prox_values[i] * 0.14
                case 4: #rightmost sensor
                    global_controller_target_speeds[0] -= prox_values[i] * 0.07
            # if prox_values[i] > 0:
            #     if i < 2: #left sensors
            #         brake_left += prox_values[i] * 0.5 * ROTATION_SPEED
            #     elif i > 2: #right sensors
            #         brake_right += prox_values[i] * 0.5 * ROTATION_SPEED
            #     else: #center sensor
            #         brake_left += prox_values[i] * 0.3 * ROTATION_SPEED
            #         brake_right += prox_values[i] * 0.3 * ROTATION_SPEED
        
        #Calculate final speeds
        left_speed = global_controller_target_speeds[0]
        right_speed = global_controller_target_speeds[1]

        #print(f"Prox values: {prox_values}, Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")

        #Set motor speeds
        await self.node.set_variables({
            "motor.left.target": [round(10000)], #CHANGE
            "motor.right.target": [round(10000)], #CHANGE
        })


    
    def global_controller(self, path, pose):
        """
        Make the robot goes straight towards the waypoint and corrects its angle error by breaking proportionally the wheels to the angle error.
        """
        x_wp = path[0][X]
        y_wp = path[0][Y]
        dx = x_wp - pose[X]
        dy = y_wp - pose[Y]
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - pose[THETA]

        # Normalize angle_diff to the range [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        correction = K_ERROR_ANGLE * angle_diff
        base_speed = ROTATION_SPEED
        left_speed = base_speed - correction
        right_speed = base_speed + correction

        #print(f"Angle diff: {angle_diff:.2f}, Correction: {correction:.2f}, Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")

        # Set motor speeds
        # await self.node.set_variables({
        #     "motor.left.target": [round(left_speed)],
        #     "motor.right.target": [round(right_speed)],
        # })
        return [round(left_speed), round(right_speed)]

    async def allign(self, pose, path):
        """
        Rotate the robot to face the next waypoint.
        """
        x_wp, y_wp = path[0][X], path[0][Y]
        dx = x_wp - pose[X]
        dy = y_wp - pose[Y]
        target_angle = math.atan2(dy, dx)
        target_angle -= math.pi/2  # Adjust for robot's orientation
        angle_diff = target_angle - pose[THETA]

        target_speed = ROTATION_SPEED
        if angle_diff > 0:
            target_speed *= -1

        # Set motor speeds
        await self.node.set_variables({
            "motor.left.target": [target_speed],
            "motor.right.target": [-target_speed],
        })

       #await client.sleep(1.0)  # Small delay to allow rotation

        #await self.reset()
    
    async def reset(self):
        await self.node.set_variables({
            "motor.left.target": [0],
            "motor.right.target": [0],
        })

    def alligned(self, pose, path):
        #shift referential for pose[theta] currently pose = 0rad if alligned with y axis. Shift it to be 0 when alligned with x axis.
        #pose = (pose[X], pose[Y], pose[THETA] - math.pi/2)

        x_wp = path[0][X]
        y_wp = path[0][Y]
        dx = (x_wp - pose[X])
        dy = (y_wp - pose[Y])
        target_angle = math.atan2(dy, dx)

        #shifts target angle by 90 degrees to match pose theta referential
        target_angle -= math.pi/2

        angle_diff = (target_angle - pose[THETA])
        #print(pose[THETA]*(180.0/math.pi))
        #print(target_angle*(180.0/math.pi))


        # Normalize angle_diff to the range [-pi, pi]
        #print(angle_diff)
        #angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        #print(angle_diff)

        return abs(angle_diff) < ANGLE_TOLERANCE

    def check_reached(self, path, pose):
        x_wp = path[0][X]
        y_wp = path[0][Y]
        dx = x_wp - pose[X]
        dy = y_wp - pose[Y]
        distance = (dx**2 + dy**2)**0.5

        if distance < DISTANCE_TOLERANCE:
            path.pop(0)
            return True
        return False

    # def local_controller(self):
    #     ...

    async def toggle(self):
        #Stop the robot
        await self.node.set_variables({
            "motor.left.target": [0],
            "motor.right.target": [0],
        })


async def test(path, pose):
    with ClientAsync() as client:  # dans le main

        with await client.lock() as node:  # dans le main
            # await client.lock()
            # Ajouter ce qu'on va utiliser !
            await node.wait_for_variables({"motor.left.target", "motor.right.target", "prox.horizontal"})
            motion = Motion(node)

            for pose in poses:
                await motion.follow_path(path, pose)
                await client.sleep(DT)
                #print(f"Current pose: {pose}, Remaining path: {path}")

            # await asyncio.sleep(2.0)

            # # arrêt
            # await node.set_variables({
            #     "motor.left.target": [0],
            #     "motor.right.target": [0],
            # })


if __name__ == "__main__":
    import asyncio  # dans le main
    import math
    from tdmclient import ClientAsync  # dans le main

    path = [(100, 100), (200, 200), (300, 100)]
    # simulate list of pose updates
    poses = [
        #(0, 0, 3.14/4),
        #(50, 50, (3.14/4)),
        (99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),(99, 99, (3.14/4)),
        (50, 50, 0.1),
        (50, 50, 0.1),
        (50, 50, 0)
    ]


    asyncio.run(test(path, poses))  # dans le main