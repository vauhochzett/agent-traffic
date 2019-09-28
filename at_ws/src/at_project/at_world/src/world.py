#!/usr/bin/env python

import rospy

from at_msgs.msg import PositionMsg


class World(object):
    def __init__(self):
        rospy.init_node("world")
        self.test_pos_pub = rospy.Publisher("/agent1/position", PositionMsg, queue_size=1)

    def simulate(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            pos = PositionMsg()
            pos.pos_x = 0
            pos.pos_y = 0
            pos.theta = 0
            pos.velocity = 0
            self.test_pos_pub.publish(pos)
            rospy.loginfo("Published")
            rate.sleep()


if __name__ == "__main__":
    world = World()
    world.simulate()
