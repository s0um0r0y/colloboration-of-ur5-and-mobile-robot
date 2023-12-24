#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize all  flags and parameters here
        self.robot_pose=[0,0,0]
        self.dock_aligned=True
        self.a=0
        self.a_speed=0.0  #angular speed
        self.l_speed=0.0  #linear speed
        self.a_docking=False   #angular docking
        self.l_docking=False   #linear docking 
        self.robot_pose = [0, 0, 0]
        self.goal_orientation = 0.0  # Initialize to a default value
        self.goal_distance = 0.0  # Initialize to a default value

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)


    def send_velocity_command(self, linear_x, linear_y, linear_z, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
       
    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message

        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.quaternion_array = msg.pose.pose.orientation
        orientation_list = [self.quaternion_array.x, self.quaternion_array.y, self.quaternion_array.z, self.quaternion_array.w]
        self.roll,self.pitch,self.yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = self.yaw

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        while angle>math.pi:
            angle-=2*math.pi
        while angle<-math.pi:
            angle+=2*math.pi
        
        return angle

    # Main control loop for managing docking behavior

    def controller_loop(self):
        print('controller loop')
        print('self dock aligned is',self.dock_aligned)
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.dock_aligned==False:
            if self.a_docking == False:
                # Implement control logic here for linear and angular motion
                # For exampled P-controller is enough, what is P-controller go check it out !
                # ...

                self.tar_angle=self.goal_orientation  #target angle
                self.ang_err=self.normalize_angle(self.tar_angle-self.robot_pose[2]) #angular error
                self.a_speed=0.7*self.ang_err
                self.vel_msg=Twist()
                self.vel_msg.angular.z=self.a_speed
                print(self.vel_msg)
                self.cmd_vel_pub.publish(self.vel_msg)

                if abs(self.ang_err) < 0.03:
                    self.a_docking=True
                    self.a_speed=0.0
                    self.vel_msg.angular.z = self.a_speed    
                    self.cmd_vel_pub.publish(self.vel_msg)
                    self.get_logger().info("Robot is Angullarly set for docking.")
                else:
                    self.get_logger().info("Aligning the robot Angullarly")

            else:
                print('Angular docking is completed')
            if self.a_docking == True and self.l_docking == False:
                print("starting linear docking")
                self.tar_pos=self.goal_distance
                if self.linear_correction == True:
                    self.pos_err=(self.tar_pos-self.robot_pose[1])
                else:
                    self.pos_err=-(self.tar_pos-self.robot_pose[0])
                self.x_speed=0.5*self.pos_err
                self.vel_msg_l=Twist()
                self.vel_msg_l.linear.x=self.x_speed
                print(self.vel_msg_l)
                self.cmd_vel_pub.publish(self.vel_msg_l)

                if abs(self.pos_err) < 0.2:
                    self.l_docking=True
                    self.vel_msg_l.linear.x=0.0
                    self.cmd_vel_pub.publish(self.vel_msg_l)
                    self.get_logger().info("Docking completed")
                    self.dock_aligned=True
                else:
                    self.get_logger().info("Robot is being set linearly for docking")
                  
        # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.linear_correction = request.linear_dock
        self.angular_correction = request.orientation_dock
        self.goal_orientation = request.orientation
        self.goal_distance = request.distance
        self.rack_no=request.rack_no
        self.dock_aligned=False
        self.a_docking=False
        self.l_docking=False
        # Reset flags and start the docking process
        rate = self.create_rate(2, self.get_clock())
        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")
        # Create a rate object to control the loop frequency
        # Wait until the robot is aligned for docking
        rate.sleep()
        
        # Set the service response indicating success
        response.success = True
        self.get_logger().info("Docking control initiated")
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()
    executor.shutdown()
if __name__ == '__main__':
    main()
