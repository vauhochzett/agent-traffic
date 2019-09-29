#!/usr/bin/env python3.6

import argparse
import sys
from types import SimpleNamespace

import rospy

# pylint: disable-msg=no-name-in-module
from at_msgs.msg import PositionMsg, ActionMsg


class ActionType:
    ACCELERATION       = 1
    THETA_ACCELERATION = 2


class Agent( object ):

    def __init__(self, 
            atype="car",
            max_velocity = 50,
            max_acceleration = 5,
            max_theta_acceleration = 10
            ):
        self.name = "agent1"

        self.type = atype
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_theta_acceleration = max_theta_acceleration

        self.action_pub = rospy.Publisher(f"/{self.name}/action", ActionMsg, queue_size=10)
        self.position_sub = rospy.Subscriber(f"/{self.name}/position", PositionMsg, self.callback)

        self.speed_limit = 1000

    def get_action( self, msg ):
        acceleration = 0
        if self.speed_limit > msg.velocity:
            acceleration = min(self.max_acceleration, self.speed_limit - msg.velocity)
        elif self.speed_limit < msg.velocity:
            acceleration = max(-self.max_acceleration, self.speed_limit - msg.velocity)
        theta_acceleration = 0
        
        msg = ActionMsg()
        msg.acceleration = acceleration
        msg.theta_acceleration = theta_acceleration

        return msg

    def set_speed_limit(self, limit):
        self.speed_limit = limit

################################
####### ROS routine ############

def callback(data, agent):
    action = agent.get_action( data )

    rospy.loginfo(data)
    rospy.loginfo(f"Agent: acc {action.acceleration}")
    rospy.loginfo(f"Agent: theta_acc {action.theta_acceleration}")
    
    agent.pub.publish(action)



def start(agent):
    rospy.init_node('agent1', anonymous=True)

    rate = rospy.Rate(1) # 10hz

    agent.pub = rospy.Publisher('action', ActionMsg, queue_size=10)
    agent.sub = rospy.Subscriber(
            "/agent1/position", 
            PositionMsg, 
            lambda data: callback(data, agent))

    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--atype", help="agent type (default: car)", choices=["car"], default="car")
    parser.add_argument("--max_velocity", default=50)
    parser.add_argument("--max_acceleration", default=5)
    parser.add_argument("--max_theta_acceleration", default=10)

    parser.parse_args(rospy.myargv(sys.argv)[1:])
    kargs = vars(parser.parse_args())
    print(kargs)

    agent = Agent(**kargs)

    try:
        start(agent)
    except rospy.ROSInterruptException:
        pass
