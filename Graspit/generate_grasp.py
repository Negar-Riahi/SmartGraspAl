#!/usr/bin/env python3
# coding:utf-8
import rospy
import pickle
import sys
import numpy as np
from geometry_msgs.msg import Pose
from graspit_commander import GraspitCommander

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
#from move2contact import move2contact

def randomPalmPosition(graspit, palm_pose_center, palm_orientation):
    '''
    change the position of the palm randomly
    '''
    #hand_pose is the coordinate system of the dexterous hand in graspIt,palm_pose is a self-definition coordinate system,which are both relative to the world coordinate system
    palm_pose_center = palm_pose_center  
    palm_pose_x = np.random.normal(palm_pose_center[0],0.01,10)[0] #Normal dist
    palm_pose_y = palm_pose_center[1]
    palm_pose_z = np.random.normal(palm_pose_center[2],0.01,10)[0]
    palm_pose = [palm_pose_x, palm_pose_y, palm_pose_z]   #start position of palm

    #palm_pose = [palm_pose_x, palm_pose_y, palm_pose_center[2]]   
    print ('palm_pose:', palm_pose)
    hand_pose_init, handrotm = getHandPose(palm_pose)
    print ('handrotm:', handrotm ) 
    hand_pose = Pose()
    hand_pose.position.x = hand_pose_init[0]
    hand_pose.position.y = hand_pose_init[1]
    hand_pose.position.z = hand_pose_init[2]


    hand_pose.orientation.w =  palm_orientation[0]
    hand_pose.orientation.x =  palm_orientation[1]
    hand_pose.orientation.y =  palm_orientation[2]
    hand_pose.orientation.z =  palm_orientation[3]
    rospy.sleep(1)
    graspit.setRobotPose(hand_pose)

def getHandPose(palmpose):
    wTp = np.array([[-1, 0, 0, palmpose[0]],
                    [0, -1, 0, palmpose[1]],
                    [0, 0, 1, palmpose[2]],
                    [0, 0, 0, 1]])    
    pTh = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    wTh = np.dot(wTp, pTh)
    handpose = [wTh[0][3],wTh[1][3],wTh[2][3]]
    handrotm = [wTh[0][0:3],wTh[1][0:3],wTh[2][0:3]]
    return handpose, handrotm


def writeModelfile(model_index):
    
    modelfile_path = '/home/negar/projects/Mymain/GraspIt/sim_model/model.xml'
    model_index = str(model_index)
    f = open(modelfile_path,'w')    
    content ='<?xml version="1.0" ?>\n'
    content+='<root>\n'
    content+='	<material>plastic</material>\n'  
    content+='	<geometryFile type="Inventor">'+model_index+'.stl</geometryFile>\n'
    content+='</root>'	
    f.write(content)
    f.close()


def generateGrasps(model_index,palm_pose_center):
    graspit_udf = GraspitCommander()
    graspit_udf.clearWorld()
    graspit_udf.loadWorld("huhand2")
    good_grasps = []
    bad_grasps = [] 
    for times in range(10):
        randomPalmPosition(graspit_udf,palm_pose_center)
        grasps=graspit_udf.planGrasps(search_space=SearchSpace(SearchSpace.SPACE_APPROACH), max_steps=70000)
        #grasps=graspit_udf.planGrasps(max_steps=50000)
        g = grasps.grasps
        length = len(g)
        print ('Llenght:',length)

        for i in range(0, length):
            if g[i].epsilon_quality > 0.0:
                good_grasps.append(g[i])
            else:
                bad_grasps.append(g[i])
        print ('len_goodgrasps:',len(good_grasps) )
        print ('len_badgrasps:',len(bad_grasps))

    #From the next few, it should be automated at the beginning
    file_path = '/home/negar/projects/Mymain/dataset4.0/Grasps/'
    file_path=file_path + str (model_index) +'.txt'

    #write 
    f = open(file_path, 'wb')
    pickle.dump(good_grasps, f, 0)
    print('good grasp saved')
    f.close()

def main():
    rospy.init_node('random_palm_position')
    #num = int(sys.argv[1])
    palm_pose_centers = [[0.09,0.04,-0.03],[60.0/1000,45.0/1000, -20.0/1000],[70.0/1000,40.0/1000, -0.0/1000], [40.0/1000,100.0/1000, -30/1000], [60.0/1000,70.0/1000, -0.0/1000],[0.0,0,-0.04],[40.0 / 1000, 30.0 / 1000, -70.0 / 1000], [30.0/1000,30.0/1000, -70.0/1000], [30.0/1000,30.0/1000, -70.0/1000], [30.0/1000,00.0/1000, -90.0/1000],[20.0 / 1000, 30.0 / 1000, -50.0 / 1000],[0.01,0,-0.05],[30.0/1000,30.0/1000, -50.0/1000], [5.0/1000,30.0/1000, -50.0/1000], [0,0,-0.07],[25.0/1000,30.0/1000, -50.0/1000],[0,0,-0.04],[0,0,-0.11],[0,0,-0.03], [25.0/1000,30.0/1000, -90.0/1000] ,[0,-0.02,-0.03],[35.0 / 1000, 30.0 / 1000, -40.0 / 1000],[0,0,-0.09], [35.0/1000,30.0/1000, -40.0/1000], [35.0/1000,30.0/1000, -40.0/1000],[-0.01,0.02,-0.07], [35.0/1000,30.0/1000, -70.0/1000],[0,0.05,-0.11],
                         [35.0 / 1000, 0.0 / 1000, -50.0 / 1000],[0,0,-0.025], [0,0.025,-0.05],[0,0,-0.09],[35.0/1000,0.0/1000, -110.0/1000], [35.0/1000,20.0/1000, -80.0/1000], [25.0/1000,20.0/1000, -30.0/1000],
                         [35.0 / 1000, 00.0 / 1000, -60.0 / 1000],[15.0/1000,00.0/1000, -40.0/1000],[35.0/1000,20.0/1000, -50.0/1000],[35.0/1000,20.0/1000, -50.0/1000],[35.0/1000,0.0/1000, -70.0/1000],
                         [50.0 / 1000, 50.0 / 1000, -100.0 / 1000],[15.0/1000,50.0/1000, -75.0/1000],[0,0,-0.06],[0,-0.01,-0.05],
                         [0.02, 0.035, -0.09],[0.02, 0.03, -0.07], [0.03, 0.02, -0.04], [0.03, 0.04, -0.04], [0.03, 0.008, -0.04], [0.03, 0.008, -0.04], [0.01, 0.04, -0.04], [0.028, 0.04, -0.04], [0.01, 0.070, -0.07], [0.025, 0.02, -0.065], [0.025, 0.0, -0.055], [0.025, 0.0, -0.085], [0.025, 0.0, -0.038], [0.025, 0.0, -0.08], [0.025, 0.03, -0.05], [0.025, 0.03, -0.05], [0.025, 0.045, -0.05], [0.025, 0.025, -0.05], [0.025, 0.085, -0.05], [0.005, 0.025, -0.05], [0.025, 0.01, -0.04]]
    
    # palm_pose_centers = [[0.02, 0.035, -0.04],[0.02, 0.03, -0.04], [0.03, 0.02, -0.04], [0.03, 0.04, -0.04], [0.03, 0.008, -0.04], [0.03, 0.008, -0.04], [0.028, 0.04, -0.04], [0.028, 0.04, -0.04], [0.028, 0.070, -0.04], [0.025, 0.02, -0.055], [0.025, 0.0, -0.055], [0.025, 0.0, -0.055], [0.025, 0.0, -0.038], [0.025, 0.0, -0.07], [0.025, 0.03, -0.05], [0.025, 0.03, -0.05], [0.025, 0.045, -0.05], [0.025, 0.025, -0.05], [0.025, 0.045, -0.05], [0.025, 0.025, -0.05], [0.025, 0.01, -0.04]]
    length = len(palm_pose_centers)
    #print ('input s to start:')
    #if raw_input() == 's':
   
    for i,j in enumerate(palm_pose_centers):
        if i == 0:
            continue
        model_index = 0
        print (i)
        writeModelfile(model_index)
        generateGrasps(model_index,j)
if __name__ == '__main__':
     #test2()
     #select_grasps()
     main()

