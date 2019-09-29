#!/usr/bin/env python

import rospy

# pylint: disable-msg=no-name-in-module
from at_msgs.msg import PositionMsg
from at_msgs.srv import NextMoveSrv, NewAgentSrv


class World(object):
    def __init__(self):
        rospy.init_node("world")
        self.test_pos_pub = rospy.Publisher("/agent1/position", PositionMsg, queue_size=1)
        self.nm_srv = rospy.Service("/agent1/next_move", NextMoveSrv, self.next_move)
        self.new_agent_srv = rospy.Service("/world/new_agent", NewAgentSrv, self.new_agent)

    def simulate(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            pos = PositionMsg()
            pos.pos_x = 0
            pos.pos_y = 0
            pos.theta = 0
            pos.velocity = 0
            self.test_pos_pub.publish(pos)
            rospy.loginfo("Published")
            rate.sleep()

    def next_move(self, _):
        # 0 Noop, 1 Forwards, 2 Right, 3 Backwards, 4 Left
        # TODO: Get for each currently active agent the next move and return it
        # TODO: Convert next_move to next_positions
        return { "msg_id": 0, "move": 1 }

    def new_agent(self, _):
        """ Register a new agent. (Receives empty message.) """
        # TODO Create unique ID
        # TODO Add to internal list a new agent
        # TODO Return its new name
        return "agent1"


if __name__ == "__main__":
    world = World()
    world.simulate()
