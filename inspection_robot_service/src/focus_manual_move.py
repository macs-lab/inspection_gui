#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped,Pose,TransformStamped, TwistStamped
import geometry_msgs.msg
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from rclpy.clock import ROSClock
from inspection_srvs.srv import MoveToPose
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import rosbag2_py

import numpy as np
from std_msgs.msg import Float64
import math
import csv
import os

MOVING_DISTANCE = 0.02
SPEED = -0.25 # neg is forward. 0.2 and under is too slow, circular motion on robot is visible

IDLE = 0
FEEDBACK = 1

class PoseStampedCreator(Node):

    def __init__(self):
        super().__init__('pose_stamped_creator')

        self.state = FEEDBACK

        # Create a client to the service
        self.pose_client = self.create_client(MoveToPose, '/inspection/move_to_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Hello world.')

        # Publisher to see where the robot is going
        self.pose_publisher = self.create_publisher(PoseStamped, 'published_pose', 10)

        # TF stuff 
        self.tf_buffer = Buffer() # store and manipulate transformations
        self.listener = TransformListener(self.tf_buffer, self)

        # srv request (target_pose)
        self.req = MoveToPose.Request()   

        # Create the twist client
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.twist_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.twist_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('twist service not available, waiting again...')
        self.get_logger().info('Twist says hello.')
        self.twist_req = Trigger.Request()
        self.async_twist_call_result = self.twist_client.call_async(self.twist_req)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.twist_callback)

        # Dynamic reconfigure twist_started
        self.declare_parameter('twist_started', False, descriptor=ParameterDescriptor(dynamic_typing=True)) # dynamic_typing=True makes the parameter dynamically reconfigurable
        self.declare_parameter('x_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_speed', SPEED, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_speed', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare twist_speed as a parameter
        self.declare_parameter('x_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('y_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('z_twist_angular', 0.0, descriptor=ParameterDescriptor(dynamic_typing=True)) # declare z_twist_angular as a parameter
        self.twist_started = self.get_parameter('twist_started').value
        self.x_twist_speed = self.get_parameter('x_twist_speed').value
        self.y_twist_speed = self.get_parameter('y_twist_speed').value
        self.z_twist_speed = self.get_parameter('z_twist_speed').value
        self.x_twist_angular = self.get_parameter('x_twist_angular').value
        self.y_twist_angular = self.get_parameter('y_twist_angular').value
        self.z_twist_angular = self.get_parameter('z_twist_angular').value
        self.add_on_set_parameters_callback(self.on_parameter_change) # call callback when parameter changes
        ###########################

        # Sobel initialize
        self.focus_pose_dict = {}
        self.focus_value_sub = self.create_subscription(
            Float64,
            'inspection/perception/focus_value',
            self.focus_value_callback,
            10)
        self.curr_focus_value = 0
        self.curr_max_fv = None
        ###########################

        self.tf_wt = TransformStamped()
        self.tf_timer = self.create_timer(0.1, self.on_timer)    

    # Move to the pose we received
    def on_timer(self):
        if self.state == IDLE:
            self.save_focus_pose_dict_to_csv()
        elif self.state == FEEDBACK:
            self.simple_feedback()
            self.get_logger().info('Feedback running')

    def send_request_world(self,pose_map):
        curr_time = rclpy.time.Time()
        tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', curr_time)
        
        start_pose_map = PoseStamped()
        start_pose_map.header.frame_id = 'world'
        start_pose_map.pose.position.x = tf_wt.transform.translation.x
        start_pose_map.pose.position.y = tf_wt.transform.translation.y
        start_pose_map.pose.position.z = tf_wt.transform.translation.z
        start_pose_map.pose.orientation = tf_wt.transform.rotation

        # Send the transformed pose as the request
        target_pose_map = PoseStamped()
        target_pose_map.header.frame_id = 'world'
        target_pose_map.pose = pose_map
        self.req.target_pose = target_pose_map

        # publish destination pose to topic 'published_pose'
        self.pose_publisher.publish(target_pose_map)

        # Call the service
        req = MoveToPose.Request()
        req.target_pose = target_pose_map
        req.start_pose = start_pose_map

        # Call the service
        req = MoveToPose.Request()
        req.target_pose = target_pose_map
        req.start_pose = start_pose_map

        future = self.pose_client.call_async(req)
        future.add_done_callback(self.response_callback_world)
        return
    
    def response_callback_world(self, future):
        try:
            response = future.result()
            if response.done:
                self.get_logger().info('Final Service completed successfully.')
                self.state = IDLE # State change needs to occur after moving to pose with max focus
                return
            else:
                self.get_logger().error('Final Service failed.')
        except Exception as e:
            self.get_logger().error('Final Service call failed %r' % (e,))
    
    # on_parameter_change is a new method that gets called whenever a parameter changes. It updates self.twist_started with the new value of the twist_started parameter
    def on_parameter_change(self, parameters):
        for parameter in parameters:
            if parameter.name == 'twist_started':
                self.twist_started = parameter.value
            elif parameter.name == 'x_twist_speed':
                self.x_twist_speed = parameter.value
            elif parameter.name == 'y_twist_speed':
                self.y_twist_speed = parameter.value
            elif parameter.name == 'z_twist_speed':
                self.z_twist_speed = parameter.value
            elif parameter.name == 'x_twist_angular':
                self.x_twist_angular = parameter.value
            elif parameter.name == 'y_twist_angular':
                self.y_twist_angular = parameter.value
            elif parameter.name == 'z_twist_angular':
                self.z_twist_angular = parameter.value
        return SetParametersResult(successful=True)
    
    def twist_callback(self):
        if not self.twist_started:
            return
        
        msg = TwistStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = 'tool0'
        msg.twist.linear.x = self.x_twist_speed
        msg.twist.linear.y = self.y_twist_speed
        msg.twist.linear.z = self.z_twist_speed
        msg.twist.angular.x = self.x_twist_angular 
        msg.twist.angular.y = self.y_twist_angular 
        msg.twist.angular.z = self.z_twist_angular 
        self.twist_publisher.publish(msg)
    
    def focus_value_callback(self, msg):
        self.curr_focus_value = msg.data
        # self.get_logger().info('Focus value: "%s"' % self.curr_focus_value)
        self.save_curr_focus_pose()

    def save_curr_focus_pose(self):
        try:
            time_save_dict = rclpy.time.Time()
            # In general, you would use ROSClock().now().to_msg() when you need the current time as a ROS2 message, 
            # rclpy.time.Time() when you need it as a Python object for calculations or comparisons.
            tf_wt = self.tf_buffer.lookup_transform('world', 'tool0', time_save_dict)
            
            pose_world = PoseStamped()
            pose_world.header.frame_id = 'world'
            pose_world.pose.position.x = tf_wt.transform.translation.x
            pose_world.pose.position.y = tf_wt.transform.translation.y
            pose_world.pose.position.z = tf_wt.transform.translation.z
            pose_world.pose.orientation = tf_wt.transform.rotation

            # Save the current focus value and its associated pose
            self.focus_pose_dict[self.curr_focus_value] = pose_world
            self.focus_pose_dict[self.curr_focus_value] = {'pose': pose_world, 'timestamp': time_save_dict}

        except TransformException as ex:
            self.get_logger().info(
                f'Could not fine tune {ex}')
            return

    def simple_feedback(self):
        self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.twist_started = self.get_parameter('twist_started').value

        # Current and Offset pose to move forward
        curr_time_fb = rclpy.time.Time()
        tf_wt_fb = self.tf_buffer.lookup_transform('world', 'tool0', curr_time_fb)

        current_pose = PoseStamped()
        current_pose.header.frame_id = 'world'
        current_pose.pose.position.x = tf_wt_fb.transform.translation.x
        current_pose.pose.position.y = tf_wt_fb.transform.translation.y
        current_pose.pose.position.z = tf_wt_fb.transform.translation.z
        current_pose.pose.orientation = tf_wt_fb.transform.rotation
        
        if not hasattr(self, 'offset_pose'):
            self.offset_pose = PoseStamped()
            self.offset_pose.header = current_pose.header
            self.offset_pose.pose.position.x = current_pose.pose.position.x + MOVING_DISTANCE # not sure why it's x, but tested and this is what worked
            self.offset_pose.pose.position.y = current_pose.pose.position.y 
            self.offset_pose.pose.position.z = current_pose.pose.position.z
            self.offset_pose.pose.orientation = current_pose.pose.orientation

        if self.focus_pose_dict:
            self.curr_max_fv = max(self.focus_pose_dict.keys())
            max_focus_pose = self.focus_pose_dict[self.curr_max_fv]['pose'].pose

            # Keep moving at the specified speed until current x-position reaches offset x-position
            if self.offset_pose.pose.position.x <= current_pose.pose.position.x:
                self.set_parameters([rclpy.parameter.Parameter('twist_started', rclpy.parameter.Parameter.Type.BOOL, False)])
                self.twist_started = self.get_parameter('twist_started').value

                self.send_request_world(max_focus_pose)
                time.sleep(0.5) # Wait for the robot to move to the max focus pose. Necessary, otherwise sometimes the robot doesn't move to the max focus pose
                # State change to IDLE placed in send_request_world, otherwise sometimes the robot doesn't move to the max focus pose
                self.get_logger().info(f'MAX FOCUS VALUE: {self.curr_max_fv}')

        return
    
    def save_focus_pose_dict_to_csv(self):
        # Define the directory and file name
        directory = '/home/macs/ws_moveit2/src/inspection_robot_service'
        file_path = os.path.join(directory, 'focus_pose_dict.csv')
        
        # Ensure the directory exists
        os.makedirs(directory, exist_ok=True)
        
        with open(file_path, 'w', newline='') as csvfile:
            # Extract the keys for the header
            fieldnames = ['focus_value','position','quaternion'] #['focus_value'] + list(next(iter(self.focus_pose_dict.values())).keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for focus_value, data in self.focus_pose_dict.items():
                # row = {'focus_value': focus_value}
                # row.update(data)
                pose = data['pose'].pose
                position = f"{pose.position.x},{pose.position.y},{pose.position.z}"
                quaternion = f"{pose.orientation.x},{pose.orientation.y},{pose.orientation.z},{pose.orientation.w}"
                row = {
                    'focus_value': focus_value,
                    'position': position,
                    'quaternion': quaternion
                }
                writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    pose_stamped_client = PoseStampedCreator()
    try:
        rclpy.spin(pose_stamped_client)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()