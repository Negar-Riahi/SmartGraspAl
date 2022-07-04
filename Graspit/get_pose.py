# coding:utf-8

import pickle
import numpy as np
import math
import re
from geometry_msgs.msg import Pose
from graspit_interface.msg import (
    Body,
    Energy,
    GraspableBody,
    Grasp,
    Planner,
    Robot,
    SearchContact,
    SearchSpace,
    SimAnnParams,
    PlanGraspsAction,
    PlanGraspsGoal
)

def get_pose(pose):
    
    #return: Transformation matrix of palm coordinate system relative to world coordinate system
    
    orientation = pose.orientation
    position = pose.position

    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    wRh = quat2rotm(quat)

    trans = np.array([position.x, position.y, position.z])
    trans.transpose()
    trans.shape = (3,1)

    wTh = np.concatenate((wRh, trans), axis=1)
    temp = np.array([0, 0, 0, 1])
    temp.shape = (1,4)
    wTh = np.concatenate((wTh, temp), axis=0)  
    '''
    hTp = np.array([[1, 0, 0, 0.068],
                    [0, 1, 0, -0.020],
                   [0, 0, 1, 0.012],
                    [0, 0, 0, 1]])            #Transformation matrix of palm coordinate system relative to hand coordinate system
    '''
    hTp = np.array([[1, 0, 0, -0.06],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])          #Transformation matrix of palm coordinate system relative to hand coordinate system
    
    wTp = np.dot(wTh, hTp)
    return wTp  

def get_hand_pose(pose):
    
    #return:hand pose in graspit
    
    orientation = pose.orientation
    position = pose.position

    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    wRp = quat2rotm(quat)

    trans = np.array([position.x, position.y, position.z])
    trans.transpose()
    trans.shape = (3, 1)

    wTp = np.concatenate((wRp, trans), axis=1)
    temp = np.array([0, 0, 0, 1])
    temp.shape = (1, 4)
    wTp = np.concatenate((wTp, temp), axis=0)  #Transformation matrix of hand relative coordinate system to the world coordinate system in graspit
    pTh = np.array([[1, 0, 0, -0.068],
                    [0, 1, 0, 0.020],
                    [0, 0, 1, -0.012],
                    [0, 0, 0, 1]]) 
    wTh = np.dot(wTp, pTh)
    return wTh  


def get_wTo(file_path):
    
    #return: wTo, Transformation matrix of the object coordinate system relative to the world coordinate system
    
    wTo = np.array([[0,0,-1],
                    [-1,0,0],
                    [0,1,0],
                    [0,0,0]])#  read from the sdf file of the objects
    with open(file_path,'r') as f:
        line = f.readline()
        while 'pose' not in line:
            line = f.readline()
    trans = list(map(float, re.findall(r"-?\d+\.?\d*", line)))[:3]
    trans.append(1)
    trans = np.array(trans)
    trans.transpose()
    trans.shape = (4,1)
    wTo = np.concatenate((wTo,trans),axis=1)
    return wTo



def pose2patch(pose,wTo):
    
   
    #return:  center position and angle of patch
    

    iTc = np.array([[554.254691191187, 0.0, 320.5, -0.0],
                    [0.0, 554.254691191187, 240.5, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0,0,0,1]]) 

    csimTc = np.array([[0,0,1,0],
                       [-1,0,0,0],
                       [0,-1,0,0],
                       [0,0,0,1]])   
    #
    # wTcsim = np.array([[-0.9991, 0.0005, -0.0433, 0.8632],
    #                 [-0.0007, -1, 0.0054, 0],
    #                 [-0.0433, 0.0054, 0.9990, 0.2859],
    #                 [0, 0, 0, 1]])                     

    wTcsim = np.array([[-1, 0.000, -0.0, 0.8632],
                    [-0.000, -1, 0.0, 0],
                    [-0.0, 0.0, 1,0.2859 ],
                    [0, 0, 0, 1]])  #test


    wTc = np.dot(wTcsim,csimTc)
    cTw = np.linalg.inv(wTc)
    wTo = wTo

    wTp = np.dot(wTo, pose)
    cTp = np.dot(cTw, wTp)
    # iTp = np.dot(iTc, cTp)
    iTw = np.dot(iTc, cTw)

    w_origin_p = wTp[:, 3]
    i_origin_p = np.dot(iTw, w_origin_p)
    i_origin_p = i_origin_p / i_origin_p[2]
    # center = iTp[:,3]/iTp[2,3]
    center = i_origin_p
    tan_angle = (wTp[2,0]/wTp[1,0])
    angle = math.degrees(math.atan(tan_angle))

    return center,angle


def quat2rotm(quat):
    '''
    :param quat: Quaternion(x,y,z,w)
    :return: Rotation matrix
    '''
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    # R = [[2*(w**2+x**2)-1, 2*(x*y-w*z), 2*(x*z+w*y)],
    #      [2*(x*y+w*z), 2*(w**2+y**2)-1, 2*(y*z-w*x)],
    #      [2*(x*z-w*y), 2*(y*z+w*x), 2*(w**2+z**2)-1]]
    R = np.array([[w**2+x**2-y**2-z**2, 2*(x*y-w*z), 2*(x*z+w*y)],
         [2*(x*y+w*z), w**2-x**2+y**2-z**2, 2*(y*z-w*x)],
         [2*(x*z-w*y), 2*(y*z+w*x), w**2-x**2-y**2+z**2]])
    return R