#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ eYRC#CL#2277 ]
# Author List:		[ Soumo Roy,Joel J Viju,Rohan Raj,Anirudh Singareddy ]
# Filename:		    task3a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]

import rclpy
import sys
import cv2
import math,time
import tf2_ros
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

def calculate_rectangle_area(coordinates):
    x1,y1=coordinates[0]
    x2,y2=coordinates[1]
    x3,y3=coordinates[2]

    wid=np.sqrt((x1-x2)**2+(y1-y2)**2)
    ht=np.sqrt((x2-x3)**2+(y2-y3)**2)

    area=wid*ht

    return area,wid

class aruco_tf(Node):

    def __init__(self):
        super().__init__('aruco_tf_publisher')                                       


        #camera topics are present here and may need to be modified
        self.color_cam_sub=self.create_subscription(Image,'/camera/color/image_raw',self.colorimagecb,10)
        self.depth_cam_sub=self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',self.depthimagecb,10)
        image_processing_rate=0.4                                                    
        self.bridge=CvBridge()                                                        
        self.tf_buffer=tf2_ros.buffer.Buffer()                                        
        self.listener=tf2_ros.TransformListener(self.tf_buffer,self)
        self.br=tf2_ros.TransformBroadcaster(self)         
        time.sleep(2)                
        self.timer=self.create_timer(image_processing_rate,self.process_image)     
        
        self.cv_image = None                                                           
        self.depth_image = None                                                         

    def detect_aruco(self,image):
        aruco_area_threshold = 1500
        cam_mat=np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
        dist_mat=np.array([0.0,0.0,0.0,0.0,0.0])
        size_of_aruco_m=0.15

        self.center_aruco_list=[]
        self.distance_from_rgb_list=[]
        self.angle_aruco_list=[]
        self.width_aruco_list=[]
        self.ids=[]
        self.rvec_list=[]
        self.tvec_list=[]

        # kernel2=np.ones((5,5),np.float32)/25
        # blur_image=cv2.filter2D(src=image,ddepth=-1,kernel=kernel2)
        # blur_image=cv2.GaussianBlur(image,(5,5),0)
        
        #image=unsharp_mask(image=image)

        #image = cv2.flip(image, 0)

        gray_image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow(gray_image)
        # cv2.imshow('Aruco Markers',gray_image)
        # cv2.aruco.drawDetectedMarkers(gray_image,corners)
        aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters=cv2.aruco.DetectorParameters()
        
        parameters.minDistanceToBorder =  7
        parameters.cornerRefinementMaxIterations = 149
        parameters.minOtsuStdDev= 4.0
        parameters.adaptiveThreshWinSizeMin= 7
        parameters.adaptiveThreshWinSizeStep= 49
        parameters.minMarkerDistanceRate= 0.014971725679291437
        parameters.maxMarkerPerimeterRate= 30.075976700411534 
        parameters.minMarkerPerimeterRate= 0.1524866841549599 
        parameters.polygonalApproxAccuracyRate= 0.05562707541937206
        parameters.cornerRefinementWinSize= 9
        parameters.adaptiveThreshConstant= 9.0
        parameters.adaptiveThreshWinSizeMax= 369
        parameters.minCornerDistanceRate= 0.09167132584946237
        
        corners,marker_ids,reject=cv2.aruco.detectMarkers(gray_image,aruco_dict,parameters=parameters)
        cv2.aruco.drawDetectedMarkers(gray_image,corners)
        # cv2.imshow("gray_scale",gray_image)
        print("marker_ids:", marker_ids)

        for i in range(len(marker_ids)):  
            coordinates=corners[i][0]
            area,width=calculate_rectangle_area(coordinates)

            if area>=aruco_area_threshold:
                c_x=np.mean(coordinates[:,0])
                c_y=np.mean(coordinates[:,1])
                self.center_aruco_list.append((c_x,c_y))
                rvec,tvec,_ = cv2.aruco.estimatePoseSingleMarkers(corners[i],size_of_aruco_m,cam_mat,dist_mat)
                self.rvec_list.append(rvec)
                self.tvec_list.append(tvec)
                tvec = tvec.squeeze(1)
                distance_from_rgb = tvec[0,2]
                self.distance_from_rgb_list.append(distance_from_rgb)
                self.angle_aruco_list.append(rvec)
                self.width_aruco_list.append(width)
                self.ids.append(marker_ids[i][0])
                cv2.aruco.drawDetectedMarkers(image,corners)
                cv2.drawFrameAxes(image,cam_mat,dist_mat,rvec,tvec,size_of_aruco_m)

    def depthimagecb(self, data):
        try:
            self.depth_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("failed to convert depth image: %s" % str(e))
            return

    def colorimagecb(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("failed to convert color image: %s" % str(e))
            return

    def process_image(self):
     try :
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        self.detect_aruco(self.cv_image)
        print("ids:", self.ids)

        for i in range(len(self.ids)):
            marker_id = self.ids[i]
            distance_from_rgb=self.distance_from_rgb_list[i]       
            angle_aruco=self.angle_aruco_list[i][0][0]

            cv_img=R.from_rotvec(angle_aruco).as_euler('xyz',degrees=True)
            send_to_ros=np.array([-cv_img[0]-90,-cv_img[1],cv_img[2]])
            a=R.from_euler('yzx',send_to_ros, degrees=True)
            b=R.from_euler('z',90,degrees=True)
            r=a*b
            quat=r.as_quat()

            distance_from_rgb/=1000

            cX,cY=self.center_aruco_list[i][:2]
            x=distance_from_rgb*(sizeCamX-cX-centerCamX)/focalX
            y=distance_from_rgb*(sizeCamY-cY-centerCamY)/focalY
            z=distance_from_rgb

            marker_frame_vec=np.array([x,y,z])
            # print(f"shape of{marker_frame_vec.shape}")

            angle_y_rad=(math.pi)/2
            angle_z_rad=-(math.pi)/2

            rotation_matrix_y=np.array([[np.cos(angle_y_rad),0,np.sin(angle_y_rad)],[0,1,0],[-np.sin(angle_y_rad),0,np.cos(angle_y_rad)]])
            rotation_matrix_z=np.array([[np.cos(angle_z_rad),-np.sin(angle_z_rad),0],[np.sin(angle_z_rad),np.cos(angle_z_rad),0],[0,0,1]])

            rot_vec=self.rvec_list[i]
            trans_vec=self.tvec_list[i]

            rotation_matrix_from_rvec=R.from_rotvec(rot_vec.reshape(3,))
            rotation_matrix_from_rvec=rotation_matrix_from_rvec.as_matrix()

            final_vector=np.dot(rotation_matrix_from_rvec,marker_frame_vec)
            cam_frame_vector=trans_vec+final_vector
            cam_frame_vector=cam_frame_vector.reshape(-1,1)

            rotated_cam_frame_vector=np.dot(rotation_matrix_y,np.dot(rotation_matrix_z,cam_frame_vector))
            cv2.circle(self.cv_image,(int(cX),int(cY)),5,(0,0,255),-1)

            transform_stamped=TransformStamped()
            transform_stamped.header.stamp=self.get_clock().now().to_msg()
            transform_stamped.header.frame_id='camera_link'
            transform_stamped.child_frame_id='CL#2277_cam_{}'.format(marker_id)
            transform_stamped.transform.translation.x=float(rotated_cam_frame_vector[0])
            transform_stamped.transform.translation.y=float(rotated_cam_frame_vector[1])
            transform_stamped.transform.translation.z=float(rotated_cam_frame_vector[2])
            transform_stamped.transform.rotation.w=float(quat[3])
            transform_stamped.transform.rotation.x=float(quat[0])
            transform_stamped.transform.rotation.y=float(quat[1])
            transform_stamped.transform.rotation.z=float(quat[2])
            print("transfrom of camera link:",transform_stamped)
            self.br.sendTransform(transform_stamped)

            try:
                base_to_camera = self.tf_buffer.lookup_transform('base_link','CL#2277_cam_{}'.format(marker_id),rclpy.time.Time())
            except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                continue

            transform_stamped.header.stamp=self.get_clock().now().to_msg()
            transform_stamped.header.frame_id='base_link'
            transform_stamped.child_frame_id='CL#2277_base_{}'.format(marker_id)
            transform_stamped.transform.translation.x=base_to_camera.transform.translation.x
            transform_stamped.transform.translation.y=base_to_camera.transform.translation.y
            transform_stamped.transform.translation.z=base_to_camera.transform.translation.z
            transform_stamped.transform.rotation.w=base_to_camera.transform.rotation.w
            transform_stamped.transform.rotation.x=base_to_camera.transform.rotation.x
            transform_stamped.transform.rotation.y=base_to_camera.transform.rotation.y
            transform_stamped.transform.rotation.z=base_to_camera.transform.rotation.z
            # print("transfrom of base link:",transform_stamped)
            self.br.sendTransform(transform_stamped)
        cv2.imshow('Aruco Markers',self.cv_image)
        cv2.waitKey(1) 
     except Exception as e:
         print(e)


def main():

    rclpy.init(args=sys.argv)                                      
    node = rclpy.create_node('aruco_tf_process')                   
    node.get_logger().info('Node created: Aruco tf process')        
    aruco_tf_class = aruco_tf()
    
    try:

        rate=aruco_tf_class.create_rate(500)
        rclpy.spin(aruco_tf_class)
        while rclpy.ok:
            rate.sleep()

    except KeyboardInterrupt:
        pass                                      

    aruco_tf_class.destroy_node()                                   
    rclpy.shutdown()                                                

if __name__ == '__main__':
    main()