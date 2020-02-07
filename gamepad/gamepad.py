import math
import evdev
import rospy
from ctrl_pkg.msg import ServoCtrlMsg
from time import time

ROS_RATE = 10   # 10hz
TIME_DIFF = 1.0/ROS_RATE
THROTTLE_MAX = 0.7
SQRT2 = math.sqrt(2.0)


def scale(val, src, dst):
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def scale_stick(value):
    return scale(value, (0, 255), (-1.0, 1.0))


if __name__ == '__main__':
    rospy.init_node('gamepad_node', disable_signals=True)
    pub_manual_drive = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size=10)
    msg = ServoCtrlMsg()
    # rate = rospy.Rate(ROS_RATE)
    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    gamepad = evdev.InputDevice(devices[0].fn)
    rospy.loginfo(gamepad)

    last_time = time() - TIME_DIFF
    angle = 0.0
    throttle = 0.0

    for event in gamepad.read_loop():
        # print(evdev.categorize(event))
        if event.type == 3:      # Analog stick
            now = time()
            if now - last_time < TIME_DIFF:
                continue
            if event.code == 0:  # X axis
                if throttle != 0.0:
                    x_axis = scale_stick(event.value)
                    angle = - 2.0 / math.pi * math.asin(max(1.0, x_axis/throttle))
            if event.code == 1:  # Y axis
                y_axis = scale_stick(event.value)
                throttle = THROTTLE_MAX * math.sqrt(y_axis*y_axis + angle*angle) / SQRT2
                throttle = - math.copysign(throttle, y_axis)
            try:
                if not rospy.is_shutdown():
                    msg.angle = angle
                    msg.throttle = throttle
                    pub_manual_drive.publish(msg)
                    rospy.loginfo(msg)
                    last_time = now
            except rospy.ROSInterruptException:
                print("ROS exit")
                exit(0)
