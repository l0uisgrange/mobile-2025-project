from src.vision import *

## Initialization
cap = start_camera()

## Main loop
while True:
    # Stop command
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting with command 'q'")
        break
    frame = get_frame(cap)


## Stop
stop_camera(cap)