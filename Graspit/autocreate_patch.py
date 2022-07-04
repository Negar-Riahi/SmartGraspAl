from get_pose import (
     get_pose,
     get_wTo,
     pose2patch,
     test_pose2patch,
)
import pickle
import math
from PIL import Image
import os
def get_patch(depth_path, num, *args):
    '''

    :param depth_path: File location
    :param num: Rotation angle, greater than or equal to 0, less than 90, not applicable to 90
    :param args:The center position and size of the patch
    :return: patch
    '''
    img = Image.open(depth_path)
    img_size = img.size
    radians = math.radians(num)

    # The coordinates of the center point of the patch after the image is rotated, clockwise is positive
    x = int(round((args[0][0]-img_size[0]/2)*math.cos(radians)-(args[0][1]-img_size[1]/2)*math.sin(radians)+img_size[0]/2))
    y = int(round((args[0][0]-img_size[0]/2)*math.sin(radians)+(args[0][1]-img_size[1]/2)*math.cos(radians)+img_size[1]/2))
    img2 = img.rotate(-num)  # clockwise is positive
    patch = img.crop((x-args[1][0]/2, y-args[1][1]/2, x+args[1][0]/2, y+args[1][1]/2))
    return patch

if __name__ == '__main__':

    for n in range (65):
        if n==13:
            print('model number=',n)
            #pose_file= '/home/negar/projects/Mymain/dataset4.0/modified_poses/'+str(n)+'.txt'
            pose_file='/home/negar/projects/Mymain/result_comparison/human/Graspit/pose/'+str(n)+'.txt'
            sdf_path='/home/negar/projects/Mymain/Gazebo/sdf_file/o' + str(n) + '.sdf'
            depth_path='/home/negar/projects/Mymain/simulation/new_depthes/'+str(n)+'.jpg'
            #depth_path='/home/negar/projects/Mymain/Gazebo/o5new1.jpg'

            f = open(pose_file, 'rb')
            poses = pickle.load(f)
            f.close()
            patch_size = [128, 64]
            wTo = get_wTo(sdf_path)
            print(wTo)
            for i,j in enumerate(poses):
                #j=j.pose

                pose = get_pose(j)
                center, angle = pose2patch(pose, wTo)
                img = get_patch(depth_path,angle,center,patch_size)
                #dir_path='/home/negar/projects/Mymain/dataset4.0/patches/'+str(n)+'/'
                dir_path='/home/negar/projects/Mymain/result_comparison/human/Graspit/patches/'+str(n)+'/'
                os.makedirs(os.path.dirname(dir_path), exist_ok=True)
                path =  dir_path + str(i) + '.jpeg'
                img.save(path)
                #print('grasp number=',i)
