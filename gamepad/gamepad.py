from evdev import InputDevice, categorize, ecodes
import evdev
import rospy
from std_msgs.msg import String
from ctrl_pkg.msg import ServoCtrlMsg
from time import time

ROS_RATE = 10   # 10hz
TIME_DIFF = 1.0/ROS_RATE
THROTTLE_MAX = 0.7


def scale(val, src, dst):
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


def scale_stick(value):
    return scale(value, (0, 255), (-1.0, 1.0))


if __name__ == '__main__':

    devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
    gamepad = evdev.InputDevice(devices[0].fn)
    print(gamepad)

    pub_manual_drive = rospy.Publisher('manual_drive', ServoCtrlMsg, queue_size=10)
    msg = ServoCtrlMsg()
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(ROS_RATE)
    last_time = time() - TIME_DIFF
    angle = 0
    throttle = 0

    for event in gamepad.read_loop():
        # print(categorize(event))
        if event.type == 3:      # Analog stick
            if event.code == 0:  # X axis
                angle = scale(event.value)
            if event.code == 1:  # Y axis
                throttle = THROTTLE_MAX * scale(event.value)
            now = time()
            if now - last_time < TIME_DIFF:
                continue
            try:
                if not rospy.is_shutdown():
                    # movement = "angle: %f, throttle: %f".format(angle, throttle)
                    # print(movement)
                    msg.angle = angle
                    msg.throttle = throttle
                    pub_manual_drive.publish(msg)
                    rospy.loginfo(msg)
                    last_time = time()
            except rospy.ROSInterruptException:
                print("ROS exit")
                exit(0)


