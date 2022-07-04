#save the manually chosen rectangle for reject samples that couldnt detect the object or chose a non_optimal 
# grasp -> 114 samples
import cv2
import numpy as np
import math

def data_handle(data_label_1,data_label_2):
    #prepare the data
    if data_label_1 < 10 :
        data_label_1 = str(0)+str(data_label_1)
    else:
        data_label_1 = str(data_label_1)

    if data_label_2 < 10 :
        data_label_2 = str(0)+str(data_label_2)
    else:
        data_label_2 = str(data_label_2)
     # E:\N\Arshad\4TH\Grasp rectangle CNN\chosen GR_txt
    path_rects=r'E:\N\Arshad\4TH\Grasp rectangle CNN\chosen GR_txt'+'\\'+data_label_1+'\pcd'+data_label_1+data_label_2+'cpos.txt'
    return  path_rects, data_label_1, data_label_2

def load_rectangles(path_rects):
#load grasping rectangles
    xy_data = []
    for line in open(path_rects).readlines():
        xy_str = line.split(' ')
        xy_data.append([float(xy_str[0]),float(xy_str[1])])

    xy_data = np.array(xy_data).reshape(int(len(xy_data)/4),8)
    scale=0.5
    index=0
    x1 = (((xy_data[index][0])-100)*scale)
    x2 = (((xy_data[index][2])-100)*scale)
    x3 = (((xy_data[index][4])-100)*scale)
    x4 = (((xy_data[index][6])-100)*scale)

    y1 = (((xy_data[index][1])-80)*scale)
    y2 = (((xy_data[index][3])-80)*scale)
    y3 = (((xy_data[index][5])-80)*scale)
    y4 = (((xy_data[index][7])-80)*scale)
    
    optimal_rec=[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
    return optimal_rec

if __name__ == '__main__':
    #dlabel_1
    #dlabel_2
    for dlabel_1 in range(9,10):
        for dlabel_2 in range (31,32):
            path_rects, data_label_1, data_label_2=data_handle(dlabel_1,dlabel_2)
            rec=load_rectangles(path_rects)
            print(rec)
            path=r'E:\N\Arshad\4TH\Grasp rectangle CNN\Chosen GR-crop and resize_txt'+'\\'+'\pcd'+data_label_1+data_label_2+'cpos.txt'
            with open(path, 'w') as x:
                for sub_list in rec:
                    for item in sub_list:  
                        x.write(str(item) + ' ')
                    x.write('\n')