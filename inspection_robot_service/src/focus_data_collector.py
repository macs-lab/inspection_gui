#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import String


class FocusDataCollector(Node):

    def __init__(self):
        super().__init__('focus_data_collector')
        self.metric_publisher = self.create_publisher(
            String, 'focus_metric', 10)
        
        


def main(args=None):
    rclpy.init(args=args)

    focus_data_collector = FocusDataCollector()

    rclpy.spin(focus_data_collector)
