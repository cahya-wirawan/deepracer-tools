from evdev import InputDevice, categorize, ecodes
import rospy
from std_msgs.msg import String
from time import time

ROS_RATE = 10   # 10hz
TIME_DIFF = 1.0/ROS_RATE


if __name__ == '__main__':

    dev = InputDevice('/dev/input/event1')
    print(dev)

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(ROS_RATE)
    last_time = time() - TIME_DIFF

    for event in dev.read_loop():
        now = time()
        if now - last_time < TIME_DIFF:
            continue
        last_time = now
        if event.type == ecodes.EV_KEY:
            print(categorize(event))
            try:
                if not rospy.is_shutdown():
                    hello_str = "hello world %s" % rospy.get_time()
                    rospy.loginfo(hello_str)
                    pub.publish(hello_str)
            except rospy.ROSInterruptException:
                print("ROS exit")
                exit(0)
