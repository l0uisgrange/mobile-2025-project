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

def compute_wheel_speed(sensor_values, base_speed=200):
    """
    Compute left/right wheel speeds (Thymio speeds: -500 to +500)
    """

    left, mid_left, center, mid_right, right = sensor_values

    left_speed = base_speed
    right_speed = base_speed

