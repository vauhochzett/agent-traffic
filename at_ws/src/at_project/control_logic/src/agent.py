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

    TYPES = ["car"]
    MAX_VEL_DEFAULT = 50
    MAX_ACC_DEFAULT = 5
    MAX_THETA_DEFAULT = 10


    def __init__(self, agent_type, max_velocity, max_acceleration, max_theta_acceleration):
        self.name = "agent1"

        self.type = agent_type
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_theta_acceleration = max_theta_acceleration

        self.action_pub = rospy.Publisher(f"/{self.name}/action", ActionMsg, queue_size=10)
        self.position_sub = rospy.Subscriber(f"/{self.name}/position", PositionMsg, self.on_position)

        self.speed_limit = 1000


    def on_position(self, data):
        action = self.get_action( data )

        rospy.loginfo(data)
        rospy.loginfo(f"Agent: acc {action.acceleration} | Agent: theta_acc {action.theta_acceleration}")

        self.action_pub.publish(action)


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


    ####### ROS routine ############

    def spin(self):
        rospy.init_node('agent1', anonymous=True)
        rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # argparse stores --agent-type in field agent_type
    parser.add_argument(
        "--agent-type",
        help=f"agent type (default: {Agent.TYPES[0]})",
        choices=Agent.TYPES,
        default=Agent.TYPES[0]
    )
    parser.add_argument("--max-velocity", default=Agent.MAX_VEL_DEFAULT)
    parser.add_argument("--max-acceleration", default=Agent.MAX_ACC_DEFAULT)
    parser.add_argument("--max-theta-acceleration", default=Agent.MAX_THETA_DEFAULT)

    parser.parse_args(rospy.myargv(sys.argv)[1:])
    kwargs = vars(parser.parse_args())
    print(kwargs)

    agent = Agent(**kwargs)

    try:
        agent.spin()
    except rospy.ROSInterruptException:
        pass
