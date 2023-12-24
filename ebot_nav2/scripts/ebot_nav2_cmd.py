#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from ebot_docking_boilerplate import MyRobotDockingController
from ebot_docking.srv import DockSw  
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import time
from ebot_docking.srv import DockSw  # Import custom service message
from linkattacher_msgs.srv import AttachLink, DetachLink 
import yaml
import os

class navigation(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.callback_group = ReentrantCallbackGroup()

        # please add the path for the config.yml file here
        config_path = os.path.abspath("/home/soumoroy/colcon_ws/src/ebot_nav2/scripts/config.yaml")
        with open(config_path, "r") as config_file:
            config_data = yaml.safe_load(config_file)

        # Extracting the rack position from config.yml
        self.package_id = config_data["package_id"][0]
        self.rack_name =  config_data['position'][self.package_id-1]
        self.rack3_position = list(self.rack_name.values())
        self.rack_name=list(self.rack_name.keys())[list(self.rack_name.values()).index(self.rack3_position[0])]
        self.offset1=0.3
        self.offset2=2.114
        self.offset3=0.21
        self.offset4=0.19

        # gripper attachment service
        self.attach_cli = self.create_client(AttachLink,'/ATTACH_LINK')
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service is not available...')
        self.req = AttachLink.Request()
        self.req.model1_name = 'ebot'
        self.req.link1_name = 'ebot_base_link'
        self.req.model2_name = 'rack3'
        self.req.link2_name = 'link'
        self.detach_cli = self.create_client(DetachLink,'/DETACH_LINK')
        while not self.detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting')

        # gripper detachment service
        self.reqd=DetachLink.Request()
        self.reqd.model1_name = 'ebot'
        self.reqd.link1_name = 'ebot_base_link'
        self.reqd.model2_name = 'rack3'
        self.reqd.link2_name = 'link'
        self.client = self.create_client(DockSw,'dock_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting')


        self.odom_sub = self.create_subscription(Odometry,'odom',self.odometry_callback,10)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        self.robot_pose = [0.0,0.0,0.0] 

        # parameters used for docking
        self.linear_correction = True
        self.angular_correction = True
        self.distance = self.rack3_position[0][1]            # linear distance to dock and attach to the rack 
        self.goal_orientation = self.rack3_position[0][2]    # angular distance to dock to the rack
        self.goal_orientation2 = self.rack3_position[0][2]*2 # angular distance to dock to get aligned with arm pose
        self.distance2 = 1.08                                # linear distance to dock to the arm pose and get dettached 

        # ur5 arm pose
        self.goal_pose2 = PoseStamped()
        self.goal_pose2.header.frame_id = 'map'
        self.goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose2.pose.position.x = -0.1        
        self.goal_pose2.pose.position.y = -2.255
        self.goal_pose2.pose.position.z = 0.0
        self.goal_pose2.pose.orientation.z = 1.57
        self.goal_pose2.pose.orientation.w = 1.57 

    # docking request function
    def send_docking_request(self,linear_correction,angular_correction,distance,goal_orientation,rack_number):
        self.request = DockSw.Request()
        self.request.linear_dock = linear_correction
        self.request.orientation_dock = angular_correction
        self.request.distance = distance
        self.request.orientation = goal_orientation
        self.request.rack_no = rack_number
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self,self.future)
        self.response = self.future.result()
        if self.future.result() is not None:
            self.response = self.future.result()
            self.get_logger().info('Docking request was successful: %s' % (self.response.success))
            self.get_logger().info('Message: %s' % (self.response.message))
            self.success = True
        else:
            self.get_logger().error("service call didn't happen properly")
            self.success = False

    # getting the robot current pose it will be used to check if docking is completed or not
    def odometry_callback(self,msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.quaternion_array = msg.pose.pose.orientation
        orientation_list = [self.quaternion_array.x,self.quaternion_array.y,self.quaternion_array.z,self.quaternion_array.w]
        self.roll,self.pitch,self.yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = self.yaw

    # feedback to tell us if nav2 completed its goal and then continue to execute the rest of the code
    def feedback(self):
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.isTaskComplete()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('goal succeeded')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def nav_Calling(self):
        
        self.goal_pose1 = PoseStamped()
        self.goal_pose1.header.frame_id = 'map'
        self.goal_pose1.header.stamp = self.navigator.get_clock().now().to_msg()
        # self.goal_pose1.pose.position.x = 2.32        # rack pose 
        # self.goal_pose1.pose.position.y = -5.976
        # self.goal_pose1.pose.position.z = 0.0   
         
        # getting the intermediate pose to travel and after this docking will be initiated
        self.goal_pose1.pose.position.x=self.rack3_position[0][0]+self.offset1
        self.goal_pose1.pose.position.y=self.rack3_position[0][1]+self.offset2
        self.goal_pose1.pose.position.z = 0.0 
        self.goal_pose1.pose.orientation.z = 0.7071
        self.goal_pose1.pose.orientation.w = 0.7071

        # navigating to the intermediate coordinate close to the rack
        self.navigator.goToPose(self.goal_pose1)
        self.feedback()
        self.send_docking_request(True,self.angular_correction,self.distance,self.goal_orientation, self.rack_name)

        # docking successfuly initated
        if self.success==True:
            # waiting the execution of this script until docking script completes execution
            while not(self.robot_pose[1]<self.rack3_position[0][1]+self.offset3 and self.robot_pose[1]>self.rack3_position[0][1]+self.offset4):
                print(self.robot_pose)
                rclpy.spin_once(self)
            time.sleep(1)
            print('docking completed')

            # attaching the rack
            self.future2 = self.attach_cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future2)
            print(self.future2.result)
            print(self.future)
            print('attachment completed')
         
        # navigating to intermediate pose close to ur5
        self.navigator.goToPose(self.goal_pose2)
        self.feedback()
        print('successfully travelled to point 2')
        self.send_docking_request(False,self.angular_correction,self.distance2,self.goal_orientation2,self.rack_name)
        print('checking')

        # docking to arm pose
        while not(self.robot_pose[0]<0.89 and self.robot_pose[0]>0.88):
                rclpy.spin_once(self)
                print(self.robot_pose)
        time.sleep(1)
        print('last docking completed')

        # detaching the rack 
        self.future4 = self.detach_cli.call_async(self.reqd)
        rclpy.spin_until_future_complete(self, self.future4)
        print('detachment completed')
        
        # making the ebot to go to a pose after completion of detachment
        self.goal_pose2.pose.position.x = -2.0
        self.navigator.goToPose(self.goal_pose2)
        self.feedback()
        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator_node = navigation()
    executor = MultiThreadedExecutor()
    executor.add_node(navigator_node)
    navigator_node.nav_Calling()
    executor.spin()
    navigator_node.destroy_node()
    rclpy.shutdown()
    executor.shutdown()
    
if __name__ == '__main__':
    main()