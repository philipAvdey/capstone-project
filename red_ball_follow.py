#!/usr/bin/python3
import cv2
import numpy as np
import ros_robot_controller_sdk as rrc
import time

board = rrc.Board()

cap = cv2.VideoCapture(0)

# -----------------------------
# RED ball HSV
# -----------------------------
lower_red1 = np.array([0,120,70])
upper_red1 = np.array([10,255,255])

lower_red2 = np.array([170,120,70])
upper_red2 = np.array([180,255,255])

# -----------------------------
# SETTINGS (tuned for stability)
# -----------------------------
speed_forward = 25
speed_rotate = 15

TARGET_RADIUS = 50

DISTANCE_DEADBAND = 8
CENTER_DEADBAND = 100   # MUCH larger tolerance

COMMAND_DELAY = 0.25    # prevents rapid switching

# smoothing
radius_history = []
HISTORY_SIZE = 8

last_command_time = 0


# -----------------------------
# Motor Functions
# -----------------------------
def stop():
    board.set_motor_duty([[1,0],[2,0],[3,0],[4,0]])

def forward():
    board.set_motor_duty([[1,-speed_forward],[2,speed_forward],[3,-speed_forward],[4,speed_forward]])

def backward():
    board.set_motor_duty([[1,speed_forward],[2,-speed_forward],[3,speed_forward],[4,-speed_forward]])

def rotate_left():
    board.set_motor_duty([[1,speed_rotate],[2,speed_rotate],[3,speed_rotate],[4,speed_rotate]])

def rotate_right():
    board.set_motor_duty([[1,-speed_rotate],[2,-speed_rotate],[3,-speed_rotate],[4,-speed_rotate]])


# -----------------------------
# Radius smoothing
# -----------------------------
def smooth_radius(r):

    radius_history.append(r)

    if len(radius_history) > HISTORY_SIZE:
        radius_history.pop(0)

    return sum(radius_history)/len(radius_history)


# -----------------------------
# Detect red ball
# -----------------------------
def detect_ball(frame):

    blurred = cv2.GaussianBlur(frame,(11,11),0)
    hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv,lower_red1,upper_red1)
    mask2 = cv2.inRange(hsv,lower_red2,upper_red2)

    mask = mask1 + mask2

    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.dilate(mask,None,iterations=2)

    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:

        c = max(contours,key=cv2.contourArea)

        ((x,y),radius) = cv2.minEnclosingCircle(c)

        if radius > 5:
            return int(x),int(y),int(radius)

    return None


# -----------------------------
# COMMAND HELPER
# -----------------------------
def allow_command():
    global last_command_time

    if time.time() - last_command_time > COMMAND_DELAY:
        last_command_time = time.time()
        return True

    return False


# -----------------------------
# MAIN LOOP
# -----------------------------
print("Stable ball follower started")

while True:

    ret,frame = cap.read()
    if not ret:
        break

    result = detect_ball(frame)

    frame_width = frame.shape[1]
    center_x = frame_width/2

    if result is not None:

        x,y,radius = result
        radius = smooth_radius(radius)

        cv2.circle(frame,(x,y),int(radius),(0,0,255),3)

        x_error = x - center_x

        if allow_command():

            # -----------------------------
            # ROTATION
            # -----------------------------
            if abs(x_error) > CENTER_DEADBAND:

                if x_error > 0:
                    print("Rotate right (slow)")
                    rotate_right()

                else:
                    print("Rotate left (slow)")
                    rotate_left()

            else:

                # -----------------------------
                # DISTANCE CONTROL
                # -----------------------------
                radius_error = radius - TARGET_RADIUS

                if abs(radius_error) < DISTANCE_DEADBAND:
                    print("Hold position")
                    stop()

                elif radius_error > 0:
                    print("Slow backward")
                    backward()

                else:
                    print("Slow forward")
                    forward()

    else:
        stop()

    cv2.imshow("Ball Tracker",frame)

    if cv2.waitKey(1) == ord('q'):
        break

stop()
cap.release()
cv2.destroyAllWindows()