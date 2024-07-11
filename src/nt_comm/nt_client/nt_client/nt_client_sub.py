#!/usr/bin/env python3
#
# A client that publishes some synchronized values periodically

# import argparse
import os
from os.path import basename
import logging
import time

import ntcore
import numpy as np

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped

class NTClientSub(Node):

    def __init__(self):
        super().__init__('NT_Client_Subscribe_Node', allow_undeclared_parameters=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('NT_server_ip', "10.80.20.2"),
                ('use_sim_time', False),
                ('sampling_time', 0.1),
                ('sub_NT_name', []),
                ('msg_types', []),
                ('pub_rostopic_names', [])]
        )

        # self.glb_sp_wpnts = WpntArray()
        # self.glb_sp_wpnts_sub_ = self.create_subscription(WpntArray, '/global_waypoints/shortest_path', self.glb_sp_wpnts_cb, 10)
        # self.glb_sp_wpnts_sub_

        # self.goal_update_pub_ = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)

        logging.basicConfig(level=logging.DEBUG)
        
        # get NT Server IP address
        ip = self.get_parameter('NT_server_ip').value

        # Initialize NT4 client
        inst = ntcore.NetworkTableInstance.getDefault()

        identity = f"{basename(__file__)}-{os.getpid()}"
        inst.startClient4(identity)

        inst.setServer(ip)

        # Start a timer
        self.Ts = self.get_parameter('sampling_time').value
        self.timer = self.create_timer(self.Ts, self.periodic)

    def msg_cb(self, msg):
        
    def periodic(self):
        # TODO: periodic logic
        
        
def main(args=None):
    rclpy.init(args=args)

    node = NTClientSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    # # publish two values
    # table = inst.getTable("data")
    # pub1 = table.getDoubleTopic("1").publish()
    # pub2 = table.getDoubleTopic("2").publish()

    # i = 3

    # while True:
    #     # These values are being published fast than the server is polling
    #     pub1.set(i)
    #     pub2.set(i + 100)

    #     time.sleep(0.5)
    #     i += 1