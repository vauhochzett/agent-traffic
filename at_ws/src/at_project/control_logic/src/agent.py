#!/usr/bin/env python3.6

import argparse
import sys
from types import SimpleNamespace

import rospy

# pylint: disable-msg=no-name-in-module
from at_msgs.msg import PositionMsg, ActionMsg
from at_msgs.srv import NewAgentSrv


NEW_AGENT_SRV_PATH = "/world/new_agent"


class ActionType:
    ACCELERATION       = 1
    THETA_ACCELERATION = 2


class Agent( object ):

    TYPES = ["car"]
    MAX_VEL_DEFAULT = 50
    MAX_ACC_DEFAULT = 5
    MAX_THETA_DEFAULT = 10


    def __init__(self, agent_type, max_velocity, max_acceleration, max_theta_acceleration):
        """ Ctor """
        received_name = self.register_with_world()
        self.name = received_name

        self.type = agent_type
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_theta_acceleration = max_theta_acceleration

        rospy.init_node(self.name, anonymous=True)

        self.action_pub = rospy.Publisher(f"/{self.name}/action", ActionMsg, queue_size=10)
        self.position_sub = rospy.Subscriber(f"/{self.name}/position", PositionMsg, self.on_position)

        self.speed_limit = 1000

        rospy.loginfo(f"New agent created: {self.__dict__}")


    def register_with_world(self):
        """ Ensure the world knows about us... """
        new_agent_srv_prx = rospy.ServiceProxy(NEW_AGENT_SRV_PATH, NewAgentSrv)

        rospy.loginfo(f"Waiting for service {NEW_AGENT_SRV_PATH}...")
        new_agent_srv_prx.wait_for_service(timeout=5)

        rospy.loginfo("Registering...")
        new_agent_srv_response = new_agent_srv_prx()
        return new_agent_srv_response.agent_name


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


    def spin(self):
        # Separation of concerns
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

    filtered_argv = rospy.myargv()[1:]
    args = parser.parse_args(filtered_argv)
    kwargs = vars(args)

    agent = Agent(**kwargs)

    try:
        agent.spin()
    except rospy.ROSInterruptException:
        pass
