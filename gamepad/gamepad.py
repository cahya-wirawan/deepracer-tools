from evdev import InputDevice, categorize, ecodes
import rospy
from std_msgs.msg import String



def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':

    dev = InputDevice('/dev/input/event1')

    print(dev)
    for event in dev.read_loop():
        if event.type == ecodes.EV_KEY:
            print(categorize(event))

    try:
        talker()
    except rospy.ROSInterruptException:
        pass