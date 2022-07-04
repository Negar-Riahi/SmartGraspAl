#save optimal recatangle chosen based on COG & COR minimum distances
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
      #E:\N\Arshad\4TH\Grasp rectangle CNN\dataset\01\pcd0100r.png
      #E:\N\Arshad\4TH\Grasp rectangle CNN\dataset\01\pcd0199cpos.txt
    path_png = r'E:\N\Arshad\4TH\Grasp rectangle CNN\dataset'+'\\'+data_label_1+'\pcd'+data_label_1+data_label_2+'r.png'
    path_rects=r'E:\N\Arshad\4TH\Grasp rectangle CNN\dataset'+'\\'+data_label_1+'\pcd'+data_label_1+data_label_2+'cpos.txt'
    return path_png, path_rects, data_label_1, data_label_2


def CC(path_png):
    #find COG
    img = cv2.imread(path_png)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (21, 21), 0)
    thresh= cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)[1]
    kernel = np.ones((5,5),np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    min_area = 100
    max_area = 8000
    COG=[0,0]
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area and area<max_area:
             cv2.drawContours(img,[c], 0, (36,255,12), 2)
             M=cv2.moments(c)
             if	M["m00"]!=0:
                cX= int(M["m10"] / M["m00"])
                cY=int(M["m01"] / M["m00"])
                COG=[cX,cY]
             else:
                cX,cY=0
                COG=[cX,cY]
                print("object not found!")
        
    
    #cv2.circle(img, (cX, cY), 7, (0, 0, 255), -1)
    #cv2.putText(img, "center", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #print(f'center of gravity for object is at:{COG}') 
    return COG 

def load_rectangles(path_rects):
#load grasping rectangles
    xy_data = []
    for line in open(path_rects).readlines():
        xy_str = line.split(' ')
        xy_data.append([float(xy_str[0]),float(xy_str[1])])

    xy_data = np.array(xy_data).reshape(int(len(xy_data)/4),8)


    shift_x = 0
    shift_y = 0
    scale = 1
    COR=[]
    for i in range(len(xy_data)):

        x1 = (xy_data[i][0]-shift_x)*scale
        x2 = (xy_data[i][2]-shift_x)*scale
        x3 = (xy_data[i][4]-shift_x)*scale
        x4 = (xy_data[i][6]-shift_x)*scale

        y1 = (xy_data[i][1]-shift_y)*scale
        y2 = (xy_data[i][3]-shift_y)*scale
        y3 = (xy_data[i][5]-shift_y)*scale
        y4 = (xy_data[i][7]-shift_y)*scale
        COR.append([float((x1+x3)/2),float((y1+y2)/2)])
        #COR[i][0]=(x1+x3)/2
        #COR[i][1]=(y1+y2)/2
    #print(f'center of each of the grasping rectangels are at:{COR}')
    return COR , xy_data

#Draw the COG and the chosen grasp closest to COG
def OR_rectangle(index,xy_data):
    scale=1
    #scale=0.5
    '''
    x1 = (((xy_data[index][0])-100)*scale)
    x2 = (((xy_data[index][2])-100)*scale)
    x3 = (((xy_data[index][4])-100)*scale)
    x4 = (((xy_data[index][6])-100)*scale)

    y1 = (((xy_data[index][1])-80)*scale)
    y2 = (((xy_data[index][3])-80)*scale)
    y3 = (((xy_data[index][5])-80)*scale)
    y4 = (((xy_data[index][7])-80)*scale)
    '''
    x1 = ((xy_data[index][0])*scale)
    x2 = ((xy_data[index][2])*scale)
    x3 = ((xy_data[index][4])*scale)
    x4 = ((xy_data[index][6])*scale)

    y1 = ((xy_data[index][1])*scale)
    y2 = ((xy_data[index][3])*scale)
    y3 = ((xy_data[index][5])*scale)
    y4 = ((xy_data[index][7])*scale)
    optimal_rec=[[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
    return optimal_rec

    



if __name__ == '__main__':
    

    for dlabel_1 in range(10,11):
        for dlabel_2 in range (0,35):
            print(f'its my {dlabel_2} turn in {dlabel_1} repo')
            path_png,path_rects,data_label_1,data_label_2=data_handle(dlabel_1,dlabel_2)
            COG=CC(path_png)
            COR,xy_data=load_rectangles(path_rects)
            d=[]
             #calculate distance between COG and center of each grasping ractangle and find the minimum distance
            for i in range(len(COR)):
                d.append(math.sqrt((COG[0]-COR[i][0])**2+(COG[1]-COR[i][1])**2))
            min_index = d.index(min(d))
            rec=OR_rectangle(min_index,xy_data)

            path=r'E:\N\Arshad\4TH\Grasp rectangle CNN\unedited_txt_format'+'\\'+'\pcd'+data_label_1+data_label_2+'cpos.txt'
            with open(path, 'w') as x:
                for sub_list in rec:
                    for item in sub_list:  
                        x.write(str(item) + ' ')
                    x.write('\n')


   