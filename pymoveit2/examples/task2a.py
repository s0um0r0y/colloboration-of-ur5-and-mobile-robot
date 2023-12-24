#!/usr/bin/env python3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from std_srvs.srv import Trigger 
import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
import math,time
from threading import Thread
from pymoveit2 import MoveIt2
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from tf_transformations import euler_from_quaternion

class Transforms(Node):
    def __init__(self):
        super().__init__('transforms')
        self.buffer=Buffer(Duration(seconds=12.0))
        self.listener=TransformListener(self.buffer,self)
        self.twist_pub=self.create_publisher(TwistStamped,"/servo_node/delta_twist_cmds",10)
        self.timer1=self.create_timer(1.0,self.lookup_tf)
        self.start_servo=self.create_client(Trigger,'/servo_node/start_servo')
        
        while not self.start_servo.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("servo service is not available")
        request=Trigger.Request()
        future=self.start_servo.call_async(request)
        rclpy.spin_until_future_complete(self,future)

        # gripper magnet on service
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

        while not self.gripper_control.wait_for_service(timeout_sec=1.0):    
            self.get_logger().info('AttachLink service not available, waiting again...')

        # gripper magnet off service
        self.gripper_control_2 = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not self.gripper_control_2.wait_for_service(timeout_sec=1.0):    
            self.get_logger().info('DetachLink service not available, waiting again...')
        if future.result() is not None:
            if future.result().success:                   
                    self.get_logger().info("Servo controller started successfully.")
            else:
                    self.get_logger().error("Failed to start Servo controller:")

        # declaring the starting pose
        self.declare_parameter(
        "starting_pose",
        [
            0.0,-2.3911,2.40855,-3.14,-1.57,3.14
        ]
        )

        # declaring the drop pose
        self.declare_parameter(
        "drop_pose",
        [
            0.00,
            -2.949,
            0.9075,
            -4.206,
            -1.5708,
            3.1498768354918307
        ],
        )
       
        # declaring the right pose
        self.declare_parameter(
        "right_pose",
        [
            -1.57,-2.3911,2.40855,-3.14,-1.57,3.14
        ]
        )

        # declaring the left pose
        self.declare_parameter(
        "left_pose",
        [
            1.57,-2.3911,2.40855,-3.14,-1.57,3.14
        ]
        )

    def lookup_tf(self):
        try:
            transform1=self.buffer.lookup_transform("base_link","obj_3",Time().to_msg())
            self.translation=(transform1.transform.translation.x,transform1.transform.translation.y,transform1.transform.translation.z)
            self.rotation=(transform1.transform.rotation.x,transform1.transform.rotation.y,transform1.transform.rotation.z,transform1.transform.rotation.w)

            self.get_logger().info("successfully completed lookup transform")

            # cancelling the timer to start moveit2
            self.timer1.cancel()
            self.start_moveit2()

        except Exception as e:
            self.get_logger().warning(f"Unable to lookup transform:{e}")

    #starting moveit2
    def start_moveit2(self):
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
        node=self,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=self.callback_group,
        )
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.timer2=self.create_timer(0.15,self.servo)

    #here we are sending request to the gripper to pick up the box  
    def send_request_gripper(self,model1):
            self.req = AttachLink.Request()
            print('attachment in progress')
            self.req.model1_name=model1
            self.req.link1_name='link'
            self.req.model2_name='ur5'
            self.req.link2_name='wrist_3_link'
            time.sleep(1)
            self.future = self.gripper_control.call_async(self.req)
            time.sleep(1)
            self.response = self.future.result()
            if self.future.result() is not None:
                self.get_logger().info('Gripper request was successful: %s' % (self.response.success))
                self.get_logger().info('Message: %s' % (self.response.message))
            else:
                self.get_logger().error('Service call failed.')
    
    #here we are sending the request to the gripper to drop the box
    def send_request_degrip(self,model1):
        self.req2= DetachLink.Request()
        self.req2.model1_name=model1
        self.req2.link1_name='link'
        self.req2.model2_name='ur5'
        self.req2.link2_name='wrist_3_link'
        time.sleep(1)
        self.future2 = self.gripper_control_2.call_async(self.req2)
        time.sleep(1)
        self.response = self.future2.result()
        if self.future2.result() is not None:
                self.response = self.future2.result()
                self.get_logger().info('Degripper request was successful: %s' % (self.response.success))
                self.get_logger().info('Message: %s' % (self.response.message))
        else:
                self.get_logger().error('Service call failed.')

    #here we are moving to the drop pose
    def move_to_drop_pose(self):
        drop_pose = (
            self.get_parameter("drop_pose").get_parameter_value().double_array_value
            )

        self.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
        self.moveit2.move_to_configuration(drop_pose)
        self.get_logger().info(f"joint position movement started to drop pose")
        self.moveit2.wait_until_executed()
        time.sleep(1)
        self.get_logger().info("completed successfully to drop pose")
    
    #here we are moving to the start pose
    def move_to_starting(self):
        starting_pose = (
            self.get_parameter("starting_pose").get_parameter_value().double_array_value
            )
        self.get_logger().info(f"Moving to {{joint_positions: {list(starting_pose)}}}")
        self.moveit2.move_to_configuration(starting_pose)
        self.get_logger().info(f"joint position movement started to start pose")
 
        self.moveit2.wait_until_executed()
        time.sleep(1)
        self.get_logger().info("completed successfully to start pose")

    #here we are moving to the right pose 
    def move_to_right(self):
        right_pose = (
            self.get_parameter("right_pose").get_parameter_value().double_array_value
            )
        self.get_logger().info(f"Moving to {{joint_positions: {list(right_pose)}}}")
        self.moveit2.move_to_configuration(right_pose)
        self.get_logger().info(f"joint position movement started to right pose")

        self.moveit2.wait_until_executed()
        time.sleep(1)

        self.get_logger().info("completed successfully to right pose")

    #here we are moving to the left pose
    def move_to_left(self):
        left_pose = (
            self.get_parameter("left_pose").get_parameter_value().double_array_value
            )
        self.get_logger().info(f"Moving to {{joint_positions: {list(left_pose)}}}")
        self.moveit2.move_to_configuration(left_pose)
        self.get_logger().info(f"joint position movement started to left pose")
    
        self.moveit2.wait_until_executed()
        time.sleep(1)   
        self.get_logger().info("completed successfully to left pose")        
  
    def servo(self):
     
     try:
       
       #publishing transform between base link and end effector
       ur5_transform=self.buffer.lookup_transform(ur5.base_link_name(),ur5.end_effector_name(),Time().to_msg())
       current_orientation=[ur5_transform.transform.rotation.x,ur5_transform.transform.rotation.y,ur5_transform.transform.rotation.z,ur5_transform.transform.rotation.w]
       current_euler = euler_from_quaternion(current_orientation)  
       target_euler = euler_from_quaternion(self.rotation)
                       
       current_translation_pose = [
                    ur5_transform.transform.translation.x, 
                    ur5_transform.transform.translation.y,
                    ur5_transform.transform.translation.z
                                  ]
       
       #restricting the yaw from -pi to pi
       self.yaw = target_euler[2]-current_euler[2]
       while self.yaw < -math.pi or self.yaw >= math.pi:
            if self.yaw < -math.pi:
                self.yaw += 2 * math.pi
            elif self.yaw >= math.pi:
                self.yaw -= 2 * math.pi
       error=0.05

       if abs(self.yaw - math.pi/2)<error:
                    self.move_to_left()

       elif abs(self.yaw + math.pi/2)<error:
                    self.move_to_right()
       difference=[0.0,0.0,0.0]
       difference_sq=[0.0,0.0,0.0]
       difference[0]=self.translation[0]-current_translation_pose[0]
       difference[1]=self.translation[1]-current_translation_pose[1]
       difference[2]=self.translation[2]-current_translation_pose[2]
       difference_sq[0]=math.pow(difference[0],2)
       difference_sq[1]=math.pow(difference[1],2)
       difference_sq[2]=math.pow(difference[2],2)
       distance=math.sqrt(difference_sq[0]+difference_sq[1]+difference_sq[2])
       print('distance to target is ',distance)
     
       if distance<0.01:
                    self.send_request_gripper('box3')
                    time.sleep(1)
                    self.move_to_starting()
                    time.sleep(1)
                    self.move_to_drop_pose()
                    self.send_request_degrip('box3')
                    self.move_to_starting()

                    print('completed the process')

                    self.timer2.cancel()
       else:
                    scaling_factor = 0.8
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    for i in range(3):
                        twist_msg.twist.linear.x = difference[0] * scaling_factor
                        twist_msg.twist.linear.y = difference[1] * scaling_factor
                        twist_msg.twist.linear.z = difference[2] * scaling_factor
                    self.twist_pub.publish(twist_msg)
     
     except Exception as e:
         print(e)

def main():
    rclpy.init()
    transforms=Transforms()
    transforms.lookup_tf()
    rclpy.spin(transforms)
    transforms.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()