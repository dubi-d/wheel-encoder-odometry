#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Header, Float32
from std_srvs.srv import Empty

class WheelEncoderOdometryNode(DTROS):

    def __init__(self, node_name):
        """
        Wheel Encoder Node
        This implements basic functionality with the wheel encoders. It works assuming that the robot is controlled
        via virtual joystick, or moved forward by hand. It will produce wrong results if moved backwards by hand.
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
        self._initial_ticks = {"left": 0, "right": 0}  # ticks when this node was launched
        self._current_ticks = {"left": 0, "right": 0}

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped,
                                                       self.cb_encoder_left)
        self.sub_encoder_ticks_right = rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped,
                                                        self.cb_encoder_right)

        # Publishers
        self.pub_travelled_distance_left = rospy.Publisher("~travelled_distance_left", Float32, queue_size=1)
        self.pub_travelled_distance_right = rospy.Publisher("~travelled_distance_right", Float32, queue_size=1)

        # Service to calculate wheel radius
        self.srv_calculate_wheel_radius = rospy.Service("~calculate_wheel_radius", Empty, self.calculate_wheel_radius)

        self.log("Initialized")

    def run_publishers(self):
        r = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            self.pub_travelled_distance_left.publish(self._travelled_distance["left"])
            self.pub_travelled_distance_right.publish(self._travelled_distance["right"])
            r.sleep()

    def cb_encoder_left(self, msg):
        """
        Compute travelled distance on left encoder tick callback.
        """
        if self._encoder_resolution is None:
            # initialize variables when the first message arrives
            self._encoder_resolution = msg.resolution
            self._initial_ticks["left"] = msg.data
            self.log(f'Initial ticks left side: {self._initial_ticks["left"]}')

        self._current_ticks["left"] = msg.data
        self.update_travelled_distance("left")

    def cb_encoder_right(self, msg):
        """
        Compute travelled distance on right encoder tick callback.
        """
        if self._encoder_resolution is None:
            # initialize variables when the first message arrives
            self._encoder_resolution = msg.resolution
            self._initial_ticks["right"] = msg.data
            self.log(f'Initial ticks right side: {self._initial_ticks["right"]}')

        self._current_ticks["right"] = msg.data
        self.update_travelled_distance("right")

    def update_travelled_distance(self, side):
        self._travelled_distance[side] = 2 * np.pi * self._radius * (self._current_ticks[side]
                                            - self._initial_ticks[side]) / self._encoder_resolution


    def calculate_wheel_radius(self):
        """
        Assuming the robot moved exactly 1.5 meters since this node was started, compute the wheel radius
        based on the encoder ticks. The robot must moved via joystick or manually forward, never manually backwards.

        :return: estimated wheel radius
        """
        real_distance = 1.5  # [m]
        r_left = self._encoder_resolution * real_distance / (2 * np.pi * (self._current_ticks["left"] - self._initial_ticks["left"]))
        r_right = self._encoder_resolution * real_distance / (2 * np.pi * (self._current_ticks["right"] - self._initial_ticks["right"]))
        return 0.5 * (r_left + r_right)

if __name__ == '__main__':
    node = WheelEncoderOdometryNode(node_name='wheel_encoder_odometry_node')
    node.run_publishers()
    rospy.loginfo("wheel_encoder_odometry_node is up and running...")
