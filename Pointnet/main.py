#to prepare the files,downsample and compute their cog

import os
import open3d as o3d
import numpy as np

def replace(file):
    with open(file) as f:
        contents = f.read()
    #print(contents)

    contents = contents.replace(',', ' ')

    f = open(file, "w")
    f.write(contents)
    f.close()

def downsample(file,destination):
    #pcd = o3d.io.read_point_cloud("car.xyz")
    pcd =  o3d.io.read_point_cloud(file, format='xyzn')
    #print(pcd)
    #print(np.asarray(pcd.points))    
   
    downpcd = pcd.uniform_down_sample(10)
    add='../modelnet_downsampled/'+destination
    #add=r'E:\N\Arshad\6TH\modelnet_resampled'+ '\\'+destination
    o3d.io.write_point_cloud(add, downpcd)

def compute_cog(file,destination):

    text_array = np.loadtxt(file,delimiter=' ')
    x = text_array[:, 0]
    y = text_array[:, 1]
    z = text_array[:, 2]
    xbar=np.sum(x)/len(x)
    ybar=np.sum(y)/len(y)
    zbar=np.sum(z)/len(z)

    xbar = float("{0:.4f}".format(xbar))
    ybar = float("{0:.4f}".format(ybar))
    zbar = float("{0:.4f}".format(zbar))
    center=[xbar,ybar,zbar]
   #add='../modelnet_downsampled_cog/'+destination
    #add='../PSG/simulated_controlled_cog/'+destination
    np.savetxt(destination, center,fmt='%1.4f', newline=" ")
    #return center

def get_obj_filenames():
    obj_filelist_file = os.path.join(MODELNET40_PATH, 'filelist.txt')
    obj_filenames = [os.path.join(MODELNET40_PATH, line.rstrip()) for line in open(obj_filelist_file)]
    print('Got %d obj files in modelnet40.' % len(obj_filenames))
    return obj_filenames
    

if __name__ == "__main__":

    '''
    MODELNET40_PATH ='../modelnet40_normal_resampled'
    Downsampled_PATH='../modelnet_downsampled'
    #obj_filelist_file = os.path.join(MODELNET40_PATH, 'filelist.txt')
    new_obj_filelist_file=os.path.join(Downsampled_PATH, 'filelist.txt')     #has .xyz instead of .txt at the end

    #obj_filenames = [os.path.join(MODELNET40_PATH, line.rstrip()) for line in open(obj_filelist_file)]
    new_obj_filenames=[os.path.join(Downsampled_PATH, line.rstrip()) for line in open(new_obj_filelist_file)]

    #folder_name=[line.rstrip() for line in open(obj_filelist_file) ]
    new_folder_name=[line.rstrip() for line in open(new_obj_filelist_file) ]
    
    #folder_name = folder_name.replace('txt', 'xyz')
    #subf=os.path.join(MODELNET40_PATH,'modelnet40_shape_names.txt')
    #subf_names=[line.rstrip() for line in open(subf)]

   # for i in range(len(subf_names)):
     #   os.mkdir(os.path.join('../modelnet_downsampled_cog/', subf_names[i]))


    #for i in range(len(folder_name)):
     #   folder_name[i] = folder_name[i].replace('txt', 'xyz')

    #print(folder_name)
    
    for i in range(len(new_obj_filenames)):
       # downsample(obj_filenames[i],folder_name[i])
       compute_cog(new_obj_filenames[i],new_folder_name[i])
    '''
    for i in range(65):
        file='../PSG/simulated_controlled/'+str(i) +'.txt'
        destination='../PSG/simulated_controlled_cog/'+str(i) +'.txt'
        compute_cog(file,destination)


    