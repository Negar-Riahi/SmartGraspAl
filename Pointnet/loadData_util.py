import os
import sys
import numpy as np
import h5py
import datetime

def get_obj_filenames():
    obj_filelist_file = os.path.join(MODELNET40_PATH, 'modelnet40_test.txt')
    obj_filenames = [os.path.join(MODELNET40_PATH,'ztest', line.rstrip()) for line in open(obj_filelist_file)]
    #obj_filenames=obj_filenames+'.xyz'
    return obj_filenames

def read_files(file):

    text_array = np.loadtxt(file,delimiter=' ')
    x = text_array[:, 0]
    y = text_array[:, 1]
    z = text_array[:, 2]
    np.round(x, 4)
    np.round(y, 4)
    np.round(z, 4)
  

    return x,y,z

def read_cog(file):
    text_array = np.loadtxt(file)
    x = text_array[0]
    y = text_array[1]
    z = text_array[2]
    np.round(x, 4)
    np.round(y, 4)
    np.round(z, 4)
  

    return x,y,z


def save_h5(h5_filename, data,label, data_dtype='uint8',label_dtype='uint8'):
    h5_fout = h5py.File(h5_filename,'w')
    h5_fout.create_dataset(
            'data', data=data,
            compression='gzip', compression_opts=4,
            dtype=data_dtype,
    )
    
    h5_fout.create_dataset(
            'label', data=label,
            compression='gzip', compression_opts=1,
            dtype=label_dtype,
    )
    
    h5_fout.close()

if __name__ == "__main__":

    #MODELNET40_PATH ='../modelnet_downsampled'
    #test_PATH ='../modelnet_downsampled/airplane/airplane_0001.xyz'

    #data_dim=[9843,1000,3]   #train
    #data_dim=[2468,1000,3]    #test
    data_dim=[65,1000,3]


    #label_dim=[9843,3]       #train
    #label_dim=[2468,3]        #test
    label_dim=[65,3]
    tensor_data = np.zeros(tuple(data_dim)) 
    tensor_label= np.zeros(tuple(label_dim)) 
    #print(tensor_data.shape)
    #obj_filenames=get_obj_filenames()     # this was commented

    #x,y,z=read_files(test_PATH)
    #np.reshape(x,(1000,1))

    #print(x)
    #print(x.shape)
    #tensor_data[0,:,0]=x
    #tensor_data[0,:,1]=y
    #tensor_data[0,:,2]=z

    #print(tensor_data)
 

    '''
    for i in range (len(obj_filenames)):
        file=obj_filenames[i]
        x,y,z=read_files(file)
        np.reshape(x,(1000,1))
        np.reshape(y,(1000,1))
        np.reshape(z,(1000,1))

        tensor_data[i,:,0]=x
        tensor_data[i,:,1]=y
        tensor_data[i,:,2]=z

    # change the path now to read cog 
    
    MODELNET40_PATH ='../modelnet_downsampled_cog'
    obj_filenames_cog=get_obj_filenames()
    for i in range (len(obj_filenames_cog)):
        file=obj_filenames_cog[i]
        x,y,z=read_cog(file)
        tensor_label[i,0]=x
        tensor_label[i,1]=y
        tensor_label[i,2]=z

    
    print(tensor_label)
    h5_batch_size = 2048
    #N=9843
    N=2468

    data_dtype = 'float32'
    label_dtype = 'float32'

    # set batch buffer
    batch_data_dim = data_dim
    batch_label_dim =  label_dim
    h5_batch_data = np.zeros(batch_data_dim)
    h5_batch_label = np.zeros(batch_label_dim)
    output_filename_prefix='test_downsampled'
    for k in range(N):
        d = tensor_data[k,:,:]
        l = tensor_label[k,:]
        h5_batch_data[k%h5_batch_size, ...] = d
        h5_batch_label[k%h5_batch_size, ...] = l
        
        if (k+1)%h5_batch_size == 0 or k==N-1:
            print ('[%s] %d/%d' % (datetime.datetime.now(), k+1, N))
            print ('batch data shape: ', h5_batch_data.shape)
            h5_filename = output_filename_prefix+str('_')+str(k//h5_batch_size)+'.h5'
            begidx = 0
            endidx = min(h5_batch_size, (k%h5_batch_size)+1) 
            save_h5(h5_filename, h5_batch_data[begidx:endidx,:,:], h5_batch_label[begidx:endidx,:],data_dtype,label_dtype)
            '''

    for i in range (65):
        file='../PSG/simulated_controlled/'+str(i) +'.txt'
        x,y,z=read_files(file)
        np.reshape(x,(1000,1))
        np.reshape(y,(1000,1))
        np.reshape(z,(1000,1))

        tensor_data[i,:,0]=x
        tensor_data[i,:,1]=y
        tensor_data[i,:,2]=z

    # change the path now to read cog 

    for i in range (65):
        file='../PSG/simulated_controlled_cog/'+str(i) +'.txt'
        x,y,z=read_cog(file)
        tensor_label[i,0]=x
        tensor_label[i,1]=y
        tensor_label[i,2]=z

    
    print(tensor_label)
    h5_batch_size = 65
    #N=9843
    N=65

    data_dtype = 'float32'
    label_dtype = 'float32'

    # set batch buffer
    batch_data_dim = data_dim
    batch_label_dim =  label_dim
    h5_batch_data = np.zeros(batch_data_dim)
    h5_batch_label = np.zeros(batch_label_dim)
    output_filename_prefix='test_KIT'
    for k in range(N):
        d = tensor_data[k,:,:]
        l = tensor_label[k,:]
        h5_batch_data[k%h5_batch_size, ...] = d
        h5_batch_label[k%h5_batch_size, ...] = l
        
        if (k+1)%h5_batch_size == 0 or k==N-1:
            print ('[%s] %d/%d' % (datetime.datetime.now(), k+1, N))
            print ('batch data shape: ', h5_batch_data.shape)
            h5_filename = output_filename_prefix+str('_')+str(k//h5_batch_size)+'.h5'
            begidx = 0
            endidx = min(h5_batch_size, (k%h5_batch_size)+1) 
            save_h5(h5_filename, h5_batch_data[begidx:endidx,:,:], h5_batch_label[begidx:endidx,:],data_dtype,label_dtype)