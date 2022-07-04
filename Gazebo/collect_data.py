#!/usr/bin/env python3
#coding:utf-8
import os
import rospy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import cv2
import numpy as np
import copy
from cv_bridge import CvBridge
import sys


def points_callback(data,args):
    gen = point_cloud2.read_points(data,skip_nans = False)
    rospy.sleep(1)
    normal_points = np.array(list(gen))
    normal_points = np.reshape(normal_points, (480, 640, 4))
    normal_points[np.logical_or(np.logical_or(np.isinf(normal_points), np.isneginf(normal_points)),
                                    np.isnan(normal_points))] = 0
    normal = normal_points
    for i in xrange(3):
        normal[:, :, i] = 255. * (normal[:, :, i] - np.min(normal[:, :, i])) / \
                (np.max(normal[:, :, i]) - np.min(normal[:, :, i]))
    normal = normal.astype('uint8')
    cv2.imwrite(args[0], normal)

    curv = normal_points[:,:,3]
    curv = 255. * (curv - np.min(curv)) / (np.max(curv) - np.min(curv))
    cv2.imwrite(args[1], curv)

def rgb_callback(data,args):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data)
    file_path = args 
    cv2.imwrite(file_path, cv_image)   

def depth_callback(data):
    #print('recieved s.th !')
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data)
    cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    #file_path = '/home/projects/output/fig4.jpg'
    cv2.imwrite('o25.jpg', cv_image*255)  
     

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data)
    cv2.imwrite('/home/sim/1.jpg', cv_image)    

def load_model(sdf_path,name):
    #load model files
    file_path = sdf_path
    #name = '"model"'
    command = 'rosrun gazebo_ros spawn_model -sdf -file ' + file_path + ' -model ' + '\"' + name +'\"'
    os.system(command)

def del_model(name):
    #Delete model files
    command = 'rosservice call /gazebo/delete_model "model_name: ' +'\'' + name + '\'' +'\"'
    os.system(command)


if __name__ == '__main__':
    
   
    rospy.init_node('collect_image', anonymous = True,disable_signals=True)
    num = sys.argv[-1]
    del_model('object')
    sdf_path = '/home/negar/projects/Mymain/Gazebo/sdf_file/o' + num +'.sdf'
    #sdf_path = '/home/negar/projects/Mymain/Gazebo/sdf_file/o25.sdf'
    load_model(sdf_path,'object')
    #subscribe to image topic and save the image
    #print 'input s to start:' #Detect the object coordinate system
    #if raw_input() == 's':
     #   pass
    #else:
    #    exit()

    #rospy.sleep(1)
    depth_path = '/home/negar/projects/PointNet_Images/'+num+'.png'
    rospy.Subscriber('/camera/color/image_raw', Image, rgb_callback,depth_path)
    rospy.sleep(1)

    #rospy.Subscriber('/camera/depth/image_raw', Image,depth_callback)
    #rospy.sleep(10)

