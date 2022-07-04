#!/usr/bin/env python3
# coding:utf-8


#Main code to automatically generate grasps for all objects

import pickle
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

from generate_grasp import randomPalmPosition, writeModelfile


def generateGrasps(model_index):
    graspit_udf = GraspitCommander()
    graspit_udf.clearWorld()
    #graspit_udf.loadWorld("huhand2")
    graspit_udf.loadWorld("huhand2")
    good_grasps = []
    bad_grasps = [] 
    rank=[]
    top5=[]
    palm_pose=[[[0.14, 0.03, -0.1],[0.707,-0.707,0,0]] ,
                [[0.08, 0.03, -0.13],[+0.486212,-0.5039, +0.51384,+0.495631]] , 
                [[-0.11, 0.03, 0.14],[-0.0252753,+0.0133417,+0.720046,+0.693338]]  ,
                [[0.13, 0.03, 0.14],[-0.507779 , +0.518206 ,+0.500107,+0.472775]] 
                ]  
    for i,j in enumerate(palm_pose):
        p=j[0]
        o=j[1]
        print(p)
        print(o)
    
        for times in range(3):
            randomPalmPosition(graspit_udf,p,o)
            grasps=graspit_udf.planGrasps(search_space=SearchSpace(SearchSpace.SPACE_APPROACH), max_steps=70000)
            #grasps=graspit_udf.planGrasps(max_steps=50000)
            g = grasps.grasps
            length = len(g)
            print ('Llenght:',length)

            for i in range(0, length):
                if g[i].epsilon_quality > 0.0:
                    rank.append(g[i].epsilon_quality)
                    good_grasps.append(g[i])
                else:
                    bad_grasps.append(g[i])
            print ('len_goodgrasps:',len(good_grasps) )
            print ('len_badgrasps:',len(bad_grasps))
    sortedrank=sorted(range(len(rank)),key=rank.__getitem__,reverse=True)  #the index of top 5
    print (sortedrank)
    '''
    one=sortedrank[0] 
    two=sortedrank[1]  
    three= sortedrank[2]
    four= sortedrank[3]
    five=sortedrank[4]
    six=sortedrank[5]
    seven=sortedrank[6]
    eight=sortedrank[7]
    top5=[good_grasps[one],good_grasps[two],good_grasps[three],good_grasps[four],good_grasps[five],good_grasps[six],good_grasps[seven],good_grasps[eight] ]  #the corresponding grasps
    #top5=[good_grasps[one],good_grasps[two],good_grasps[three],good_grasps[four],good_grasps[five]]
    '''
    
    
    #print(top5)
    file_path = '/home/negar/projects/PointNet/raw_comparison2/'
    #file_path = '/home/negar/projects/PointNet/raw_grasps/'
    file_path=file_path + str (model_index) +'.txt'
    
    #write 
    #print(rank)
    f = open(file_path, 'wb')
    pickle.dump(top5, f, 0)
    print('good grasp saved')
    f.close()
    
if __name__ == '__main__':

    palm_pose_centers = [[0.09, 0.04, -0.03], [0.09, 0.045, -0.05], [0.1, 0.04, -0.03], [0.07, 0.1, -0.03], [0.09, 0.07, -0.03], [0.03, 0, -0.07], [0.07, 0.03, -0.07], [0.06, 0.03, -0.07], [0.06, 0.03, -0.07], [0.06, 0.0, -0.09], [0.05, 0.03, -0.08], [0.04, 0, -0.08], [0.06, 0.03, -0.08], [0.035, 0.03, -0.08], [0.03, 0, -0.07], 
 [0.055, 0.03, -0.08], [0.03, 0, -0.07], [0.03, 0, -0.11], [0.03, 0, -0.06], [0.055, 0.03, -0.085], [0.03, -0.02, -0.06], [0.065, 0.03, -0.07], [0.03, 0, -0.12], [0.065, 0.03, -0.07], [0.065, 0.03, -0.07], [0.02, 0.02, -0.07], [0.065, 0.03, -0.07], [0.03, 0.05, -0.11], [0.065, 0.0, -0.08], [0.03, 0, -0.055], [0.03, 0.025, -0.08], [0.03, 0, -0.12], [0.065, 0.0, -0.11],
 [0.065, 0.02, -0.08], [0.055, 0.02, -0.06], [0.065, 0.0, -0.09], [0.045, 0.0, -0.07], [0.065, 0.02, -0.08], [0.065, 0.02, -0.08], [0.065, 0.0, -0.1], [0.08, 0.05, -0.1], [0.045, 0.05, -0.075], [0.03, 0, -0.09], [0.03, -0.01, -0.08], [0.05, 0.035, -0.09], [0.05, 0.03, -0.07], [0.06, 0.02, -0.07], [0.06, 0.04, -0.07], [0.06, 0.008, -0.07], [0.06, 0.008, -0.07], [0.04, 0.04, -0.07], 
 [0.058, 0.04, -0.07], [0.04, 0.07, -0.07], [0.055, 0.02, -0.095], [0.055, 0.0, -0.085], [0.055, 0.0, -0.085], [0.055, 0.0, -0.068], [0.055, 0.0, -0.11], [0.055, 0.03, -0.08], [0.055, 0.03, -0.08], [0.055, 0.045, -0.08], [0.055, 0.025, -0.08], [0.055, 0.085, -0.08], [0.035, 0.025, -0.08], [0.055, 0.01, -0.07]]

    for i,j in enumerate(palm_pose_centers):
        if i==13:
            model_index = i
            print ('model_index=',i)
            writeModelfile(model_index)
            generateGrasps(model_index)

