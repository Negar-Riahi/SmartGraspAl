#!/usr/bin/env python3
# coding:utf-8
import rospy
import pickle
import sys
import numpy as np
from geometry_msgs.msg import Pose 
from graspit_commander import GraspitCommander
from  generate_grasp import writeModelfile , randomPalmPosition
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
    PlanGraspsGoal,

)
from graspit_interface.srv import (
    AutoGrasp,
    AutoOpen
)
def writeFile(pose, dofs, file_name):
    dof_values = dofs
    dof_values = ' '.join([str(i) for i in dof_values ])
    position = pose.position
    p_x = position.x*1000.0
    p_y = position.y*1000.0
    p_z = (position.z)*1000.0 
    
    orientation = pose.orientation
    o_x = orientation.x
    o_y = orientation.y
    o_z = orientation.z
    o_w = orientation.w
    file_name = '/home/negar/.graspit/worlds/' + file_name + '.xml'
    f = open(file_name,'w')
    world_file = '<?xml version="1.0" ?>\n'
    world_file += '<world>\n'
    world_file +='	<graspableBody>\n'
    world_file +='		<filename>../projects/Mymain/GraspIt/sim_model/model.xml</filename>\n'
    world_file +='		<transform>\n'
    world_file +='			<fullTransform>(+1 +0 +0 +0)[+0 +0 +0]</fullTransform>\n'
    world_file +='		</transform>\n'
    world_file +='	</graspableBody>\n'
    world_file +='	<robot>\n'
    world_file +='		<filename>models/robots/HumanHand/HumanHand16DOF.xml</filename>\n'
    world_file +='		<dofValues>'
    world_file += dof_values+'</dofValues>\n'
    world_file +='		<transform>\n'
    world_file +='			<fullTransform>'
    world_file +='(' + str(o_w) +' '+ str(o_x)+ ' '  + str(o_y)+' ' + str(o_z)+ ')'
    world_file +='['+ str(p_x) +' '+ str(p_y) +' '+ str(p_z) + ']</fullTransform>\n'
    world_file +='		</transform>\n'
    world_file +='	</robot>\n'
    world_file +='</world>\n'
    
    f.write(world_file)
    f.close()

def write_file(grasp, file_name):
    dof_values = grasp.dofs
    dof_values = ' '.join([str(i) for i in dof_values ])
    position = grasp.pose.position
    p_x = position.x*1000
    p_y = position.y*1000
    p_z = position.z*1000
    
    orientation = grasp.pose.orientation
    o_x = orientation.x
    o_y = orientation.y
    o_z = orientation.z
    o_w = orientation.w
    file_name = '/home/negar/.graspit/worlds/' + file_name + '.xml'
    f = open(file_name,'w')
    world_file = '<?xml version="1.0" ?>\n'
    world_file += '<world>\n'
    world_file +='	<graspableBody>\n'
    world_file +='		<filename>../projects/Mymain/GraspIt/sim_model/model.xml</filename>\n'
    world_file +='		<transform>\n'
    world_file +='			<fullTransform>(+1 +0 +0 +0)[+0 +0 +0]</fullTransform>\n'
    world_file +='		</transform>\n'
    world_file +='	</graspableBody>\n'
    world_file +='	<robot>\n'
    world_file +='		<filename>models/robots/HumanHand/HumanHand16DOF.xml</filename>\n'
    world_file +='		<dofValues>'
    world_file += dof_values+'</dofValues>\n'
    world_file +='		<transform>\n'
    world_file +='			<fullTransform>'
    world_file +='(' + str(o_w) +' '+ str(o_x)+ ' '  + str(o_y)+' ' + str(o_z)+ ')'
    world_file +='['+ str(p_x) +' '+ str(p_y) +' '+ str(p_z) + ']</fullTransform>\n'
    world_file +='		</transform>\n'
    world_file +='	</robot>\n'
    world_file +='</world>\n'
    
    f.write(world_file)
    f.close()

def testMoveDof():

    rospy.init_node('random_palm_position')
    grasps_path ='/home/negar/projects/Mymain/Simulation_data/robo.txt'
    #grasps_path ='/home/negar/projects/Mymain/simulation/grasps/poses/25.txt'
    file_path='/home/negar/projects/Mymain/Simulation_data/poses/optimalposes1.txt'
    num=1
    f = open(grasps_path,'rb')
    grasps = pickle.load(f)
    f.close()
    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
    dofs = []
    poses = []
    for index,i in enumerate(grasps):
        if index==0:
            print ('index:',index)
            print ('i:',i)
            write_file(i,world_file)
            graspit_udf.clearWorld()
            graspit_udf.loadWorld(world_file)
            print ('input m for mod' )
            temp = input()
            vel=[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
            if temp == 'm':
                old_dofs = graspit_udf.getRobot().robot.dofs
                print('old dofs=',old_dofs)
                print('go??')
                if input()=='m':
                    pass
           
                final_dofs = [0.0, 1.57, 1.57, 1.57, 1.57, 1.57, 0.0, 1.57, 1.57, 0,  1.57, 1.57,0.0, 0.0, 0.0,0.0]
                final_dofs[0] = i.dofs[0]
                final_dofs[3] = i.dofs[3]
                final_dofs[6] = i.dofs[6]
                final_dofs[9] = i.dofs[9]
                final_dofs[12] = i.dofs[12]
                final_dofs[13] = i.dofs[13]
                #print('final dofs=',final_dofs)
                #move2contact(final_dofs,graspit_udf)
                graspit_udf.moveDOFToContacts(final_dofs,vel,False)

                new_dofs = graspit_udf.getRobot().robot.dofs
                print('new dofs=',new_dofs)
                print ('if modify finished: input f')
                if input()=='f':
                    break
        '''
            while True:
                graspit_udf.toggleAllCollisions(True)
                final_dofs = [0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0,  1.57, 1.57,0.0, 0.0, 1.57,1.74]
                final_dofs[0] = i.dofs[0]
                final_dofs[3] = i.dofs[3]
                final_dofs[6] = i.dofs[6]
                final_dofs[9] = i.dofs[9]
                final_dofs[12] = i.dofs[12]
                final_dofs[13] = i.dofs[13]

                move2contact(final_dofs,graspit_udf)
                q = graspit_udf.computeQuality()
                print ('quality:' , q)
                print ('if modify finished: input f')
                graspit_udf.toggleAllCollisions(False)
                if input()=='f':
                    break
        
'''



def M_viewGrasps():

    rospy.init_node('random_palm_position')
    grasps_path ='/home/negar/projects/Mymain/dataset/Grasps/5.txt'
    #grasps_path ='/home/negar/projects/Mymain/simulation/grasps/poses/25.txt'
    #file_path='/home/negar/projects/Mymain/Simulation_data/poses/optimalposes1.txt'
    num=5
    f = open(grasps_path,'rb')
    grasps = pickle.load(f)
    f.close()
    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
    dofs = []
    poses = []
    for index,i in enumerate(grasps):
        print ('index:',index)
        print ('i:',i)
        write_file(i,world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        print('next?')
        temp = input()
        if temp == 'n':
            continue
        '''
        print ('if next and save:input n if next and no save:input d if done:input q' )
        temp = input()
        if temp == 'n':
            poses.append(i.pose)
            dofs.append(i.dofs)
        elif temp =='d':
            continue
        elif temp== 'q':
            break
    #f = open(file_path, 'wb')
    #pickle.dump(poses, f, 0)
        '''
def move2contact(final_dofs,graspit):
  
    vel=[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
    graspit.moveDOFToContacts(final_dofs,vel,False)
   


def select_grasps():
    '''
    Modify the grasps and select them out
    ''' 
    rospy.init_node('random_palm_position')
    num = sys.argv[1]
    grasps_path = '/home/negar/projects/Mymain/result_comparison/human/Graspit/grasps/'+ num +'.txt'
    f = open(grasps_path,'rb')
    grasps = pickle.load(f)
    f.close()
    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
    #poses_path = '/home/negar/projects/Mymain/dataset4.0/modified_poses/'+ num +'.txt'
    #dofs_path = '/home/negar/projects/Mymain/dataset4.0/modified_dofs/'+ num + '.txt'
    poses_path = '/home/negar/projects/Mymain/result_comparison/human/Graspit/pose/'+ num +'.txt'
    dofs_path = '/home/negar/projects/Mymain/result_comparison/human/Graspit/dof/'+ num +'.txt'
    try:
        f = open(poses_path, 'rb')
        poses = pickle.load(f)
        f.close()
        f = open(dofs_path, 'rb')
        dofs = pickle.load(f)
        f.close()
    except:
        print ('error')
        dofs = []
        poses = []
    print (len(dofs),len(poses),len(grasps) )
    for index,i in enumerate(grasps):

        write_file(i,world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        graspit_udf.toggleAllCollisions(False)
        q = graspit_udf.computeQuality()
        print ('quality:' , q)
        print ('if modify:input m; if del:input d; if do nothing: input c')
        temp = input()
        if temp == 'm':
            while True:
                graspit_udf.toggleAllCollisions(True)
                final_dofs = [0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0,  1.57, 1.57,0.0, 0.0, 1.57,1.74]
                final_dofs[0] = i.dofs[0]
                final_dofs[3] = i.dofs[3]
                final_dofs[6] = i.dofs[6]
                final_dofs[9] = i.dofs[9]
                final_dofs[12] = i.dofs[12]
                final_dofs[13] = i.dofs[13]

                if input()=='m':
                    pass
                move2contact(final_dofs,graspit_udf)
                q = graspit_udf.computeQuality()
                print ('quality:' , q)
                print ('if modify finished: input f   if u wanna discard the whole thing : input d')
                graspit_udf.toggleAllCollisions(False)
                temp1 = input()
                if temp1=='f':
                    new_dofs = graspit_udf.getRobot().robot.dofs
                    new_pose = graspit_udf.getRobot().robot.pose                
                    poses.append(new_pose)
                    dofs.append(new_dofs)
                    break
                elif temp1=='d':
                    break
          
        elif temp =='c' :
            poses.append(i.pose)
            dofs.append(i.dofs)
        else:
            continue
        f = open(poses_path, 'wb')
        pickle.dump(poses, f, 0)
        f.close() 
        f = open(dofs_path, 'wb')
        pickle.dump(dofs, f, 0)
        f.close() 
    print (len(dofs),len(poses),len(grasps)  )
def push():
    rospy.init_node('random_palm_position')

    poses_path = '/home/negar/projects/Mymain/result_comparison/human/GDPN/pose/13_1.txt'
    f = open(poses_path,'rb')
    poses = pickle.load(f)
    f.close()

    dofs_path='/home/negar/projects/Mymain/result_comparison/human/GDPN/dofs/13_1.txt'
    f = open(dofs_path,'rb')
    dofs = pickle.load(f)
    f.close()
    writeModelfile(13)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()


    for index,i in enumerate(poses):

        writeFile(i,dofs[index],world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        graspit_udf.toggleAllCollisions(False)
        q = graspit_udf.computeQuality()
        print ('quality:' , q)
        print ('if modify:input m; if del:input d; if do nothing: input c')
        temp = input()
        if temp == 'm':
            while True:
                new_dofs = graspit_udf.getRobot().robot.dofs
                new_dofs=list(new_dofs)
                graspit_udf.toggleAllCollisions(True)
                final_dofs = [0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0,  1.57, 1.57,0.0, 0.0, 1.57,1.74]
                final_dofs[0] = new_dofs[0]
                final_dofs[3] = new_dofs[3]
                final_dofs[6] = new_dofs[6]
                final_dofs[9] = new_dofs[9]
                final_dofs[12] = new_dofs[12]
                final_dofs[13] = new_dofs[13]

                if input()=='m':
                    pass
                move2contact(final_dofs,graspit_udf)
                q = graspit_udf.computeQuality()
                print ('quality:' , q)
                print ('if modify finished: input f   if u wanna discard the whole thing : input d')
                graspit_udf.toggleAllCollisions(False)
                temp1 = input()
                if temp1=='f':
                    '''
                    new_dofs = graspit_udf.getRobot().robot.dofs
                    new_pose = graspit_udf.getRobot().robot.pose                
                    poses.append(new_pose)
                    dofs.append(new_dofs)
                    '''
                    break
                elif temp1=='d':
                    break
          
        elif temp =='c' :
            poses.append(i.pose)
            dofs.append(i.dofs)
        else:
            continue
        '''
        f = open(poses_path, 'wb')
        pickle.dump(poses, f, 0)
        f.close() 
        f = open(dofs_path, 'wb')
        pickle.dump(dofs, f, 0)
        f.close() 
        '''
    #print (len(dofs),len(poses),len(grasps)  )

def test_grasps():
    '''
    Check the grasp to make sure it is correct
    ''' 
    rospy.init_node('random_palm_position')
    num = sys.argv[1]
    #poses_path = '/home/negar/projects/PointNet/raw_comparison/pose/' + num +'.txt'
    poses_path = '/home/negar/projects/PointNet/pose/' + num +'.txt'

    #poses_path = '/home/negar/projects/Mymain/dataset4.0/modified_poses/' + num +'.txt'
    #poses_path = '/home/negar/projects/Mymain/result_comparison/cup/Graspit/pose/'+ num +'.txt'
    f = open(poses_path,'rb')
    poses = pickle.load(f)
    f.close()
    #dofs_path = '/home/negar/projects/PointNet/raw_comparison/dof_predicted/' + num +'.txt'
    dofs_path = '/home/negar/projects/PointNet/dof/' + num +'.txt'


    #dofs_path = '/home/negar/projects/Mymain/dataset4.0/modified_dofs/' + num +'.txt'
    #dofs_path = '/home/negar/projects/Mymain/result_comparison/cup/Graspit/dof/'+ num +'.txt'
    f = open(dofs_path,'rb')
    dofs = pickle.load(f)
    f.close()

    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
  
    print (len(dofs),len(poses))
    for index,i in enumerate(poses):
        writeFile(i,dofs[index],world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        #z=i.position.z
        #print('z=',z)
        q = graspit_udf.computeQuality()
        print ('quality:' , q)
        print ('for next hit c')
        temp = input()
        if temp =='c' :
            continue
 
    print( len(dofs),len(poses)   )

def view_predicted():
    '''
    Check the grasp to make sure it is correct
    ''' 
    rospy.init_node('random_palm_position')
    num = sys.argv[1]
    #num2=sys.argv[2]
    poses_path = '/home/negar/projects/Mymain/result_comparison/human/Graspit/pose/'+ num +'.txt'
    #poses_path = '/home/negar/projects/PointNet/raw_comparison/pose/' + num +'.txt'
    #poses_path = '/home/negar/projects/PointNet/pose/' + num +'.txt'
    
    f = open(poses_path,'rb')
    poses = pickle.load(f)
    f.close()
    dofs_path='/home/negar/projects/Mymain/result_comparison/cup/GDPN/dof_pred/61_3.txt'
    #dofs_path='/home/negar/projects/PointNet/raw_comparison/dof_predicted/'+ num +'.txt'
    #dofs_path='/home/negar/projects/PointNet/result_comparison/'+ num +'.txt'
   
    f = open(dofs_path,'rb')
    dofs = pickle.load(f)
    f.close()

    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
    graspit_udf.toggleAllCollisions(False)

    print (len(dofs),len(poses))
    dof=[]
    pose=[]
    for index,i in enumerate(poses):
        writeFile(i,dofs[index],world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        z=i.position.z
        print('z=',z)
        try:
            q = graspit_udf.computeQuality()
            print ('quality:' , q)
            print('not grasped modify? hit m :')
            temp = input()
            if temp =='m' :
                while True:
                    graspit_udf.toggleAllCollisions(False)
                    final_dofs = [0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0.0, 1.57, 1.57, 0,  1.57, 1.57,0.0, 0.0, 1.57,1.74]
                    final_dofs[0] = i.dofs[0]
                    final_dofs[3] = i.dofs[3]
                    final_dofs[6] = i.dofs[6]
                    final_dofs[9] = i.dofs[9]
                    final_dofs[12] = i.dofs[12]
                    final_dofs[13] = i.dofs[13]
                    if input()=='m':
                        pass
                    move2contact(final_dofs,graspit_udf)
                    q = graspit_udf.computeQuality()
                    print ('quality:' , q)
                    print ('if modify finished: input f   if u wanna discard the whole thing : input d')
                    graspit_udf.toggleAllCollisions(False)
                    temp1 = input()
                    if temp1=='f':
                        new_dofs = graspit_udf.getRobot().robot.dofs
                        new_pose = graspit_udf.getRobot().robot.pose                
                        pose.append(new_pose)
                        dof.append(new_dofs)
                        break
                    elif temp1=='d':
                        break
        except:
            print('hand is in obj :(' )
 

    #dofs_path='/home/negar/projects/Mymain/predict/test7/modofs/'+ num+'.txt'
    #dofs_path='/home/negar/projects/Mymain/result_comparison/cuboid/GDPN/dofs/42_3.txt'
    #poses_path = '/home/negar/projects/Mymain/result_comparison/cuboid/GDPN/pose/42_3.txt'
        '''
    f = open(dofs_path, 'wb')
    pickle.dump(dof, f, 0)
    f.close() 
    f = open(poses_path, 'wb')
    pickle.dump(pose, f, 0)
    f.close() 
        '''
    print( len(dofs),len(dof) )

def auto_grasp():  #Graspit autograsp fuction
    rospy.init_node('random_palm_position')
    num = sys.argv[1]
    poses_path = '/home/negar/projects/PointNet/raw_comparison2/pose/' + num +'.txt'

    #poses_path = '/home/negar/projects/Mymain/dataset4.0/modified_poses/' + num +'.txt'
    #poses_path = '/home/negar/projects/Mymain/result_comparison/cup/Graspit/pose/'+ num +'.txt'
    f = open(poses_path,'rb')
    poses = pickle.load(f)
    f.close()
    
    dofs_path = '/home/negar/projects/PointNet/raw_comparison2/dof/' + num +'.txt'
    #dofs_path = '/home/negar/projects/Mymain/dataset4.0/modified_dofs/' + num +'.txt'
    #dofs_path = '/home/negar/projects/Mymain/result_comparison/cup/Graspit/dof/'+ num +'.txt'
    f = open(dofs_path,'rb')
    dofs = pickle.load(f)
    f.close()
    writeModelfile(num)
    world_file = 'huhand2'
    graspit_udf = GraspitCommander()
  
    print (len(dofs),len(poses))
    for index,i in enumerate(poses):
        writeFile(i,dofs[index],world_file)
        graspit_udf.clearWorld()
        graspit_udf.loadWorld(world_file)
        graspit_udf.toggleAllCollisions(False)
        try:
            print('open?')
            temp= input()
            if temp =='o' :
             graspit_udf.autoOpen()
             graspit_udf.toggleAllCollisions(True)
            print('close?')
            temp= input()
            if temp =='p' :
                graspit_udf.autoGrasp()            
            q = graspit_udf.computeQuality()
            print ('quality:' , q)
            print ('for next hit c')
            temp = input()
            if temp =='c' :
                continue
        except:
            print('invalid pose:)')
            continue


if __name__ == '__main__':
    select_grasps()
    #testMoveDof()
    #M_viewGrasps()
    #test_grasps()
    #auto_grasp()
    #view_predicted()
    #push()