###############################################################################
## Author: Team Supply Bot
## Edition: eYRC 2019-20
## Instructions: Do Not modify the basic skeletal structure of given APIs!!!
###############################################################################


######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import copy

# Global Color Variables
RED_MAX = (179, 255, 255)
RED_MIN = (170, 100, 20)
GREEN_MAX = (65,255,255)
GREEN_MIN = (33,100,20)
ORANGE_MAX = (6,255,255)
ORANGE_MIN = (0,100,20)
morph_square = np.ones((5,5),np.uint8)
morph_circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
print("Change color if necessary")

# This function changes brightness and contrast(0,100)
def brightness_contrast(image, brightness, contrast):
    return cv2.addWeighted(image,1 + float(contrast) / 100.,image,0,float(brightness))
# This function returns angle between three points a,b,o or <aob
def getAngle(a,b,o):
    dirao = math.atan2(a[1] - o[1], a[0] - o[0])
    dirbo = math.atan2(b[1] - o[1], b[0] - o[0])
    aob = dirao - dirbo

    if(aob > math.pi):
        aob = aob - 2*math.pi
    if(aob < -math.pi):
        aob = aob + 2*math.pi
    
    deg = abs(math.degrees(aob))
    deg = math.ceil(deg)
    return deg
# This function returns Euclidean distance between point_a,point_b
def get_radius(point_a,point_b):
    X = point_a[0] - point_b[0]
    Y = point_a[1] - point_b[1]
    return round(math.sqrt(X*X + Y*Y))
# This function is to remove Eyantra LOGO as it is also red in color and causes trouble
def crop_arena(image):
    tmp = image
    tmp = cv2.cvtColor(tmp,cv2.COLOR_BGR2GRAY)
    _,tmp = cv2.threshold(tmp,100,255,cv2.THRESH_BINARY)
    edges = cv2.Canny(tmp,100,200)
    countours,_ = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    max_area,max_ind = 0,0
    for i in range(0,len(countours)):
        x,y,w,h = cv2.boundingRect(countours[i])
        area = w*h
        if max_area < area:
            max_area = area
            max_ind = i
    return cv2.boundingRect(countours[max_ind])

def detect_orange(image):
    tmp = cv2.inRange(image,ORANGE_MIN,ORANGE_MAX)
    countours,_ = cv2.findContours(tmp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    max_area,max_ind = 0,0
    for i in range(0,len(countours)):
        x,y,w,h = cv2.boundingRect(countours[i])
        area = cv2.contourArea(countours[i])
        if max_area < area:
            max_area = area
            max_ind = i
    M = cv2.moments(countours[max_ind])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (cX,cY)

def detect_green(image,radius,orange_center):
    image = cv2.medianBlur(image,5)
    tmp = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,tmp = cv2.threshold(tmp,110,255,cv2.THRESH_BINARY)
    green = cv2.inRange(image,GREEN_MIN,GREEN_MAX)
    remove_ref = cv2.bitwise_and(green,tmp)
    erode = cv2.erode(remove_ref,morph_square,iterations=1)
    dilate = cv2.dilate(erode,morph_circle,iterations=1)
    dilate = cv2.Canny(dilate,100,200)
    cntrs,_ = cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    selected = 0
    for i in range(0,len(cntrs)):
        x,y,w,h = cv2.boundingRect(cntrs[i])
        cX = x + w//2
        cY = y + h//2
        tmpr = get_radius(orange_center,(cX,cY)) 
        if abs(tmpr - radius) <= 15:
            selected = i
            break
    x,y,w,h = cv2.boundingRect(cntrs[selected])
    return ((x+w//2,y+h//2),cntrs[selected])

def detect_red(image):
    image = cv2.medianBlur(image,5)
    tmp = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,tmp = cv2.threshold(tmp,110,255,cv2.THRESH_BINARY)
    red = cv2.inRange(image,RED_MIN,RED_MAX)
    # cv2.imshow('redinrange',red)
    remove_ref = cv2.bitwise_and(red,tmp)
    erode = cv2.erode(remove_ref,morph_square,iterations=1)
    dilate = cv2.dilate(erode,morph_circle,iterations=1)
    cntrs,_ = cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(cntrs) > 0:
        x,y,w,h = cv2.boundingRect(cntrs[0])
        return ((x+w//2,y+h//2),cntrs[0])
    return ((0,0),None)
############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ip_image = brightness_contrast(ip_image,0,70)
    original_image = ip_image.copy()

    ip_image = cv2.medianBlur(ip_image,5)
    
    x,y,w,h = crop_arena(ip_image)
    ip_image = ip_image[y:y+h,x:x+w]

    op_image = ip_image
    
    op_image = cv2.cvtColor(ip_image,cv2.COLOR_BGR2HSV)
    
    orange_center = detect_orange(op_image)
    red_center,red_det = detect_red(op_image)
    r = get_radius(orange_center,red_center)
    green_center,green_det = detect_green(op_image,r,orange_center)
    
    if red_det is not None and green_det is not None:
        cv2.drawContours(ip_image,[red_det,green_det],-1,(255,0,0),2)
        angle = getAngle(green_center,red_center,orange_center)
        original_image[y:y+h,x:x+w] = ip_image
        cv2.putText(original_image,"Angle: "+str(angle),(10,15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        op_image = original_image
    else:
        print("Something went wrong")
    return op_image

    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Modify the image name as per instruction
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(1) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    while(ret):
        ret, frame = cap.read()
        ## display to see if the frame is correct
        cv2.imshow("window", frame)
        cv2.waitKey(int(1000/fps));
        ## calling the algorithm function
        op_image = process(frame)
        cv2.imwrite("SB#9999_task3I.jpg",op_image)


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
