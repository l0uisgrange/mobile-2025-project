#import

from tdmclient import ClientAsync # correct?
import asyncio # correct?

# ---------------------------------------------------------
# 1) Read IR sensors from Thymio
# ---------------------------------------------------------

async def read_ir_sensors(client, node):
    """
    Read the 5 front horizontal IR sensors from the Thymio.
    Sensors returned in order:
    [Left, MidLeft, Center, MidRight, Right]
    """
    await client.wait_for_variables(node)
    sensors = node["prox.horizontal"]  # This gives 7 values
    return sensors[:5]                 # We take the first 5 front sensors

# ---------------------------------------------------------
# 2) Compute wheel speeds based on sensor values
# ---------------------------------------------------------

def compute_wheel_speed(sensor_values, base_speed=200): # à changer?
    """
    Compute left/right wheel speeds (Thymio speeds: -500 to +500)
    """

    left, mid_left, center, mid_right, right = sensor_values

    left_speed = base_speed
    right_speed = base_speed

    
# Obstacle in center → slow down
    if center > 2000:          # valeur à changer?
        left_speed -= 150      # à changer?
        right_speed -= 150     # à changer?

    # Obstacle on left → turn right
    if left > 2000 or mid_left > 2000:
        left_speed -= 200
        right_speed += 200

    # Obstacle on right → turn left
    if right > 2000 or mid_right > 2000:
        left_speed += 200
        right_speed -= 200

    
    # Clamp to Thymio motor range -500 to 500
    left_speed = max(-500, min(500, left_speed))        #  valeur à changer?
    right_speed = max(-500, min(500, right_speed))      # à changer?

    return left_speed, right_speed

# ---------------------------------------------------------
# 3) Main loop
# ---------------------------------------------------------

async def main():
    client = ClientAsync()
    async with client.run_async():
        node = await client.wait_for_node()

        while True:
            # Read sensor values from Thymio
            sensors = await read_ir_sensors(client, node)

            # Compute new wheel speeds
            left_speed, right_speed = compute_wheel_speed(sensors)

            # Send speeds to robot
            await client.set_variables(
                node,
                {"motor.left.target": left_speed, "motor.right.target": right_speed}
            )

            print("Sensors:", sensors)
            print("Motors :", left_speed, right_speed)
            print("-------------------------------")

            await asyncio.sleep(0.1)


# Run
asyncio.run(main())

