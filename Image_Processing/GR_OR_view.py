#show all possible grasping rectangles (GR) alongside optimal rectangle (OR) 

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
    return path_png, path_rects


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
    img = cv2.imread(path_png)

    for i in range(len(xy_data)):

        x1 = int((xy_data[i][0]))
        x2 = int((xy_data[i][2]))
        x3 = int((xy_data[i][4]))
        x4 = int((xy_data[i][6]))

        y1 = int((xy_data[i][1]))
        y2 = int((xy_data[i][3]))
        y3 = int((xy_data[i][5]))
        y4 = int((xy_data[i][7]))
        COR.append([float((x1+x3)/2),float((y1+y2)/2)])
        cv2.line(img,(x1,y1),(x2,y2),(255, 0, 0),2)
        cv2.line(img,(x2,y2),(x3,y3),(0, 0, 255),2)
        cv2.line(img,(x3,y3),(x4,y4),(255, 0, 0),2)
        cv2.line(img,(x4,y4),(x1,y1),(0, 0, 255),2)
        cx=int((x1+x3)/2)
        cy=int((y1+y2)/2)
        cv2.putText(img, f"{i}", (cx,cy ),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #cv2.circle(img, (COG[0], COG[1]), 5, (0, 255, 0), -1)
        #COR[i][0]=(x1+x3)/2
        #COR[i][1]=(y1+y2)/2
    #print(f'center of each of the grasping rectangels are at:{COR}')
    return COR , xy_data , img

#Draw the COG and the chosen grasp closest to COG
def draw_line(index,path_png,xy_data):
    img = cv2.imread(path_png)
    x1 = int((xy_data[index][0]))
    x2 = int((xy_data[index][2]))
    x3 = int((xy_data[index][4]))
    x4 = int((xy_data[index][6]))

    y1 = int((xy_data[index][1]))
    y2 = int((xy_data[index][3]))
    y3 = int((xy_data[index][5]))
    y4 = int((xy_data[index][7]))
    cv2.line(img,(x1,y1),(x2,y2),(255, 0, 0),2)
    cv2.line(img,(x2,y2),(x3,y3),(0, 0, 255),2)
    cv2.line(img,(x3,y3),(x4,y4),(255, 0, 0),2)
    cv2.line(img,(x4,y4),(x1,y1),(0, 0, 255),2)
    cv2.circle(img, (COG[0], COG[1]), 5, (0, 255, 0), -1)
    return img

    



if __name__ == '__main__':
    #d1 = 8
    #d2 = 99
    

    #dlabel_1 = np.random.randint(d1) + 1
    #dlabel_2 = np.random.randint(d2) + 1
    dlabel_1 = 1
    dlabel_2 = 43

    path_png,path_rects=data_handle(dlabel_1,dlabel_2)
    print(path_png)
    COG=CC(path_png)
    COR,xy_data,GR=load_rectangles(path_rects)
    #print(f'number of grasping ractengles is:{len(COR)}')
    d=[]
    #calculate distance between COG and center of each grasping ractangle and find the minimum distance
    for i in range(len(COR)):
        d.append(math.sqrt((COG[0]-COR[i][0])**2+(COG[1]-COR[i][1])**2))
    print(d)
   # print(min (d))
    min_index = d.index(min(d))
    OR=draw_line(min_index,path_png,xy_data)
    Hori = np.concatenate((GR, OR), axis=1)
    cv2.imshow('GR vs OR', Hori)
    cv2.waitKey(0)







   