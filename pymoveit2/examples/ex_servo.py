#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer,TransformListener,LookupException, ConnectivityException, ExtrapolationException, TransformException
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from std_srvs.srv import Trigger
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
import math
from threading import Thread
from pymoveit2 import MoveIt2
from rclpy.time import Time

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer=Buffer(Duration(seconds=10.0))
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.translations={"obj_1": None,"obj_3": None,"obj_49": None}
        self.rotations={"obj_1": None,"obj_3": None,"obj_49": None}
        self.timer=self.create_timer(1.0,self.lookupTransforms)
        self.first_transform_received=False

    def lookupTransforms(self):
        try:
            transform1=self.tf_buffer.lookup_transform("base_link","obj_1",Time().to_msg())
            transform3=self.tf_buffer.lookup_transform("base_link","obj_3",Time().to_msg())
            transform49=self.tf_buffer.lookup_transform("base_link","obj_49",Time().to_msg())

            self.translations["obj_1"]=(transform1.transform.translation.x,transform1.transform.translation.y,transform1.transform.translation.z)
            self.translations["obj_3"]=(transform3.transform.translation.x,transform3.transform.translation.y,transform3.transform.translation.z)
            self.translations["obj_49"]=(transform49.transform.translation.x,transform49.transform.translation.y,transform49.transform.translation.z)

            self.rotations["obj_1"]=(transform1.transform.rotation.x,transform1.transform.rotation.y,transform1.transform.rotation.z,transform1.transform.rotation.w)
            self.rotations["obj_3"]=(transform3.transform.rotation.x,transform3.transform.rotation.y,transform3.transform.rotation.z,transform3.transform.rotation.w)
            self.rotations["obj_49"]=(transform49.transform.rotation.x,transform49.transform.rotation.y,transform49.transform.rotation.z,transform49.transform.rotation.w)
            
            self.printTransforms()

            if not self.first_transform_received:
                self.first_transform_received=True
                self.timer.reset()
            
        except TransformException as e:
            self.get_logger().warning(f"lookup transforms publishing failed:{e}")

    def printTransforms(self):
        for obj_id in self.translations:
            self.get_logger().info(f"Transform id : {obj_id}")
            self.get_logger().info(f"Translation : {self.translations[obj_id]}")
            self.get_logger().info(f"Rotation : {self.rotations[obj_id]}")

class ServoNode(Node):
    def __init__(self,targetPose,targetRotations,targetObjName):
        super().__init__('servo_node')
        self.moveItController=MoveMultipleJointPositions()
        callbackGroup=ReentrantCallbackGroup()
        self.attached=False

        self.tf_buffer=Buffer(Duration(seconds=10.0))
        self.tf_listener=TransformListener(self.tf_buffer,self)

        callbackGroup=ReentrantCallbackGroup()
        self.twistPub=self.create_publisher(TwistStamped,"/servo_node/delta_twist_cmds",10)

        self.targetPoses=targetPose
        self.targetRotations=targetRotations

        self.startServo()
        self.currentDestinationIndex=0
        #can be modified with a for loop
        self.ids=[int(targetObjName.split('_')[1])]
        self.distance_threshold=0.01
        self.box_done=False
        self.attaching=False

        self.cons=(math.pi)/180

        self.yaw_right_box_pose = [
            -90*(math.pi/180),-137*(math.pi/180),138*(math.pi/180),
            -180*(math.pi/180),-90*(math.pi/180),180*(math.pi/180)]

        self.yaw_left_box_pose = [
            90*(math.pi/180),-137*(math.pi/180),138*(math.pi/180),
            -180*(math.pi/180),-90*(math.pi/180),180*(math.pi/180)]

        self.home_pose=[
            0.0,-137*(math.pi/180),138*(math.pi/180),
            -180*(math.pi/180),-90*(math.pi/180),180*(math.pi/180)]
        self.startServo()
        self.timer=self.create_timer(0.02,self.servoToDestination,callbackGroup)

    def stopServoService(self):
        stopServoClient=self.create_client(Trigger,"/servo_node/stop_servo")
        if not stopServoClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Servo service not working")
        else:
            request=Trigger.Request()
            future = stopServoClient.call_async(request)
            rclpy.spin_until_future_complete(self,future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Servo halted") 
                    # change the name of node               
                    self.moveItController.dropPose()                   
                else:
                    self.get_logger().error("Servo stop failed")
            else:
                self.get_logger().error("Service halted")
    
    
    def startServo(self):
        startServoClient=self.create_client(Trigger,'/servo_node/start_servo')

        if not startServoClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("servo service is failed")
        else:
            request=Trigger.Request()
            future=startServoClient.call_async(request)
            rclpy.spin_until_future_complete(self,future)

            if future.result() != None:
                if future.result().success:
                    self.get_logger().info("Servo initiated")
                else:
                    self.get_logger().error("servo is failed")
            else:
                self.get_logger().error("Service call fail")

    def servoToDestination(self):   
        if self.currentDestinationIndex<len(self.targetPoses):
            try:
                trans=self.tf_buffer.lookup_transform(ur5.base_link_name(),ur5.end_effector_name(),Time().to_msg())
                targetPose=self.targetPoses[self.currentDestinationIndex]
                targetRotation=self.targetRotations[self.currentDestinationIndex]

                presentOrientation=[trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
                
                targetRotEuler=list(R.from_quat(targetRotation).as_euler('xyz'))
                presentOrientationEuler=list(R.from_quat(presentOrientation).as_euler('xyz'))

                yaw=targetRotEuler[2]-presentOrientationEuler[2]     

                while yaw<-math.pi:
                    yaw+=2*math.pi

                error=0.05

                if abs(yaw-math.pi/2)<error:
                    self.moveItController.moveJointConfig(self.yaw_left_box_pose)

                elif abs(yaw- -math.pi/2)<error:
                    self.moveItController.moveJointConfig(self.yaw_right_box_pose)
                               
                currentPose = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ]
                 
                #diff=[targetPose[i]-currentPose[i] for i in range(3)]
                diff = []
                for i in range(3):
                    diff.append(targetPose[i]-currentPose[i])
                # simply using function
                distance=(sum([diff[i]**2 for i in range(3)]))**0.5

                if distance<self.distance_threshold:
                    self.attaching=True

                    self.attachLinkSer()
                    
                    if (self.currentDestinationIndex%2)==1:
                        self.testFunction()                     
                        self.detachLinkSer()
                        self.moveItController.moveJointConfig(self.home_pose)
                        self.box_done=True
                        print(self.box_done) 
                        if (self.box_done):
                            self.timer.reset()           
  
                    self.currentDestinationIndex+=1
                    # print(f"Current taget index increased{self.current_target_index}")
                    
                else:
                    scaling_factor = 0.8
                    twist_msg = TwistStamped()
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    for i in range(3):
                        twist_msg.twist.linear.x = diff[0] * scaling_factor
                        twist_msg.twist.linear.y = diff[1] * scaling_factor
                        twist_msg.twist.linear.z = diff[2] * scaling_factor
                    self.twistPub.publish(twist_msg)
            except (LookupException,ConnectivityException,ExtrapolationException):
                self.get_logger().error("lookup transform failed from base_link to tool0.")


    def testFunction(self):
        self.moveItController.dropPoseafterServoing()

    def attachLinkSer(self):
        # print('timer has been cancelled')
        # Create a client for the AttachLink service
        attachLinkClient = self.create_client(AttachLink,'/GripperMagnetON')
        while not attachLinkClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AttachLink service not found')
            req = AttachLink.Request()
            req.model1_name = f'box{self.ids[int(self.currentDestinationIndex/2)]}'  # Specify the box name
            req.link1_name = 'link'
            req.model2_name = 'ur5'
            req.link2_name = 'wrist_3_link'
            # Call the AttachLink service
            future = attachLinkClient.call_async(req)
            if future.result() is not None:
                if future.result().success:               
                    self.get_logger().info("Attachment hogaya :):):):):).")
                    self.attached = True
                    self.attaching = False
                else:
                    self.get_logger().error("Attachment gaya :((((: ")
            else:
                self.get_logger().info("gg")    
                   
    def detachLinkSer(self):
        detachLinkClient = self.create_client(DetachLink, '/GripperMagnetOFF')
        while not  detachLinkClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('detachLink service gaya :((((, rukho zara!!!')
        req = DetachLink.Request()
        req.model1_name = f'box{self.ids[int(self.currentDestinationIndex/2)]}'  # Specify the box name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        # Call the AttachLink service
        future = detachLinkClient.call_async(req)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("detachment hogya !!!!.")
                self.detached = True
            else:
                self.get_logger().error("detachment gaya sab gaya :(((: %s", future.result().message)
        else:
            self.get_logger().info("gg")

class MoveMultipleJointPositions(Node):
    def __init__(self):
        super().__init__('move_multiple_joint_positions')
        self.cons=math.pi/180
        self.moveit2 = None
        self.detached = False
        self.movit_done = False 

    def moveMultipleJointPositions(self,*joint_positions):
        for i, positions in enumerate(joint_positions,start=1):
            param_name=f"joint_positions:{i}"
            self.declare_parameter(param_name,positions)

            joint_positions=self.get_parameter(param_name).get_parameter_value().double_array_value

            self.get_logger().info(f"Moving to {param_name}: {list(joint_positions)}")
            self.moveit2.move_to_configuration(joint_positions)
            self.moveit2.wait_until_executed()

    def moveJointConfig(self,joint_position):

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callbackGroup=ReentrantCallbackGroup()
        )

        executor=MultiThreadedExecutor(1)
        executor.add_node(self)
        executor=Thread(target=executor.spin,daemon=True,args=())
        executor.start()
        self.moveit2.move_to_configuration(joint_position)
        self.moveit2.wait_until_executed()
    
    def dropPoseafterServoing(self):

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callbackGroup=ReentrantCallbackGroup()
        )
       
        executor = MultiThreadedExecutor(1)
        executor.add_node(self)
        executor = Thread(target= executor.spin, daemon=True, args=())
        executor.start()
        
        joint_positions_1 = [
            0.0,
            -169*self.cons,
            52*self.cons,
            -241*self.cons,
            -90*self.cons,
            180*self.cons]

        joint_positions_2 = [
            0.0,-137*(math.pi/180),138*(math.pi/180),
            -180*(math.pi / 180), -90 * (math.pi / 180), 180 * (math.pi / 180)
        ]
        # Move to multiple joint configurations
        self.moveMultipleJointPositions(joint_positions_2, joint_positions_1)   

def main(args=None):
    rclpy.init(args=args)
    tf_listener_node = TFListener()
    while not tf_listener_node.first_transform_received:
        rclpy.spin_once(tf_listener_node,timeout_sec=1.0)
    
    tf_listener_node.get_logger().info("Tf Listener Node chal nahi raha hai")
   
    for obj_name in ["obj_1","obj_3","obj_49"]:

        test = list(tf_listener_node.translations[obj_name])
        rot = list(tf_listener_node.rotations[obj_name])

        target_rot_euler = list(R.from_quat(rot).as_euler('xyz'))

        yaw = target_rot_euler[2]

        print(yaw)

        while yaw < -math.pi:
            yaw += 2*math.pi

        error = 0.05

        if abs(yaw - math.pi/2) < error:
            test[0] -= 0.18

        elif abs(yaw - 0) < error:
            test[1] += 0.18
        elif abs(yaw - math.pi) < error:
            test[1] -= 0.18     
            
        dropPose = [-0.47, 0.12, 0.397]

        targetPoses = [tf_listener_node.translations[obj_name],test]
        targetRotations = [tf_listener_node.rotations[obj_name],tf_listener_node.rotations[obj_name]]

        servoNode = ServoNode(targetPoses,targetRotations,obj_name)
        while not servoNode.box_done :
            rclpy.spin_once(servoNode,timeout_sec=0.02)

        servoNode.get_logger().info("servo Node chal raha nahi hai")
        servoNode.destroy_node() 

    print('the code chal gaya')
    rclpy.shutdown()

if __name__ == '__main__':
    main()   