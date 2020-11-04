#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class WheelEncoderOdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(WheelEncoderOdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.log(f"Vehicle name: {self.veh_name}")

        # Get static parameters
        self._radius = rospy.get_param(f"/{self.veh_name}/kinematics_node/radius", 100)

        # init variables
        self._encoder_resolution = None
        self._travelled_distance = {"left": 0, "right": 0} # [m]
        self._previous_ticks = {"left": 0, "right": 0}
        self._executed_commands = {"left": 0, "right": 0}
        self._original_ticks = {"left": 0, "right": 0}  # ticks when this node was launched (used for radius calculation)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f"/{self.veh_name}/left_wheel_encoder_node/tick", WheelEncoderStamped,
                                                       self.cb_encoder_left)
        self.sub_encoder_ticks_right = rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped,
                                                        self.cb_encoder_right)
        self.sub_executed_commands = rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped,
                                                      self.cb_executed_commands)

        # Publishers
        self.pub_travelled_distance_left = rospy.Publisher("~travelled_distance_left", Float32)
        self.pub_travelled_distance_right = rospy.Publisher("~travelled_distance_right", Float32)

        self.log("Initialized")

    def run_publishers(self):
        r = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.pub_travelled_distance_left.publish(self._travelled_distance["left"])
            self.pub_travelled_distance_right.publish(self._travelled_distance["right"])
            r.sleep()

    def cb_encoder_left(self, msg):
        """ ToDo
        """
        self.log(msg)
        if self._encoder_resolution is None:
            # initialize variables when the first message arrives
            self._encoder_resolution = msg.resolution
            self._previous_ticks["left"] = msg.data
            self._original_ticks["left"] = msg.data
            return

        self.update_travelled_distance(msg.data, "left")

    def cb_encoder_right(self, msg):
        """ ToDo
        """
        self.log(msg)
        print(msg)
        if self._encoder_resolution is None:
            # initialize variables when the first message arrives
            self._encoder_resolution = msg.resolution
            self._previous_ticks["right"] = msg.data
            self._original_ticks["right"] = msg.data
            return

        self.update_travelled_distance(msg.data, "right")

    def cb_executed_commands(self, msg):
        """ ToDo
        """
        self._executed_commands = {"left": msg.vel_left, "right": msg.vel_right}

    def update_travelled_distance(self, ticks, side):
        self.log("Called update_travelled_distance")
        delta_distance = 2 * np.pi * self._radius * (ticks - self._previous_ticks[side]) / self._encoder_resolution
        if self._executed_commands[side] >= 0:
            self._travelled_distance[side] += delta_distance
        elif self._executed_commands[side] < 0:
            self._travelled_distance[side] -= delta_distance

    def compute_wheel_radius(self, real_distance):
        """
        Assuming the robot moved exactly real_distance meters since this node was started, compute the wheel radius
        based on the encoder ticks.

        :param real_distance: Distance the duckiebot has travelled since launching this node (moved via joystick or manually forward)
        :return: estimated wheel radius
        """
        r_left = self._encoder_resolution * real_distance / (2 * np.pi * (self._previous_ticks["left"] - self._original_ticks["left"]))
        r_right = self._encoder_resolution * real_distance / (2 * np.pi * (self._previous_ticks["right"] - self._original_ticks["right"]))
        return 0.5 * (r_left + r_right)

if __name__ == '__main__':
    node = WheelEncoderOdometryNode(node_name='wheel_encoder_odometry_node')
    node.run_publishers()
    rospy.loginfo("wheel_encoder_odometry_node is up and running...")
