#!/usr/bin/env python3.6

import rospy
from std_msgs.msg import String
import argparse 

def callback(data):
    print(data)

def listener(args):
    rospy.init_node('agent', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", help="agent type (default: car)", default="car")
    parser.parse_args()
    args = parser.parse_args()
    print(args)

    try:
        listener(args)
    except rospy.ROSInterruptException:
        pass
