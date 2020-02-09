from time import time
import math
from select import select
import evdev
import rospy
from ctrl_pkg.msg import ServoCtrlMsg
from ctrl_pkg.srv import (ActiveStateSrv,
                          EnableStateSrv,
                          NavThrottleSrv)

ROS_RATE = 20   # 20hz
TIME_DIFF = 1.0/ROS_RATE
TIME_TO_STOP = 1.0  # 1 seconds to stop the motor
throttle_max = 0.6
motor_state = True

def scale(val, src, dst):
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def scale_stick(value):
    return scale(value, (0, 255), (-1.0, 1.0))


if __name__ == '__main__':
    rospy.init_node('gamepad_node', disable_signals=True)
    pub_manual_drive = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size=10)
    enable_state_req = rospy.ServiceProxy('enable_state', EnableStateSrv)
    msg = ServoCtrlMsg()
    # rate = rospy.Rate(ROS_RATE)
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    gamepad = evdev.InputDevice(devices[0].fn)
    rospy.loginfo(gamepad)

    last_time = time() - TIME_DIFF
    angle = 0.0
    throttle = 0.0
    x_axis = 0.0
    y_axis = 0.0
    start_stop_state = False

    while True:
        # print(evdev.categorize(event))
        r, w, x = select([gamepad.fd], [], [], TIME_TO_STOP)
        if r:
            for event in gamepad.read():
                now = time()
                if event.type == 1:      # Right Button
                    if event.code == 304 and event.value == 1:  #BTN_SOUTH
                        motor_state = not motor_state
                        start_stop_state = not start_stop_state
                        enable_state_req(start_stop_state)
                        rospy.loginfo("###### START/STOP: " + str(start_stop_state) + ":" + str(motor_state))
                    elif event.code == 307 and event.value == 1:    #BTN_NORTH
                        throttle_max = min(1.0, throttle_max + 0.1)
                        rospy.loginfo("###### throttle_max: " + str(throttle_max))
                    elif event.code == 308 and event.value == 1:    #BTN_WEST
                        throttle_max = max(0.3, throttle_max - 0.1)
                        rospy.loginfo("###### throttle_max: " + str(throttle_max))
                if event.type == 3 and (event.code == 0 or event.code == 1):      # Analog stick
                    if now - last_time < TIME_DIFF:
                        continue
                    if event.code == 0:  # X axis
                        x_axis = scale_stick(event.value)
                        angle = - x_axis
                    if event.code == 1:  # Y axis
                        y_axis = scale_stick(event.value)
                        throttle = min(1.0, math.sqrt(y_axis*y_axis + x_axis*x_axis))
                        throttle = - math.copysign(throttle, y_axis)
                    if motor_state and not start_stop_state and abs(angle) >= 0.1 and abs(throttle) >= 0.1:
                        start_stop_state = True
                        enable_state_req(start_stop_state)
                        rospy.loginfo("###### START")
                    if event.code == 0 or event.code == 1:
                        if start_stop_state:
                            try:
                                if not rospy.is_shutdown():
                                    msg.angle = angle
                                    msg.throttle = throttle_max * throttle
                                    pub_manual_drive.publish(msg)
                                    # rospy.loginfo(msg)
                                    last_time = now
                            except rospy.ROSInterruptException:
                                print("ROS exit")
                                exit(0)
        else:
            rospy.loginfo("###### WILL STOP")
            if start_stop_state:
                start_stop_state = False
                enable_state_req(start_stop_state)
                rospy.loginfo("###### STOP")