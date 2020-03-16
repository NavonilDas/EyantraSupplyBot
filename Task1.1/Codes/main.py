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




########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Images'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))



def getAngle(a,b,o):
    dirao = math.atan2(a[1] - o[1], a[0] - o[0])
    dirbo = math.atan2(b[1] - o[1], b[0] - o[0])
    aob = dirao - dirbo

    if(aob > math.pi):
        aob = aob - 2*math.pi
    if(aob < -math.pi):
        aob = aob + 2*math.pi
    
    deg = abs(math.degrees(aob))
    print(deg)
    deg = round(deg*100)/100
    return deg


############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    angle = 0.00
    h,w = ip_image.shape[:2]
    center = (w//2,h//2)
    
    red = cv2.inRange(ip_image,(0,0,250),(0,0,255))
    M = cv2.moments(red)
    cx = (int)(M["m10"] / M["m00"])
    cy = (int)(M["m01"] / M["m00"])
    red_center = (cx,cy)

    green = cv2.inRange(ip_image,(0,250,0),(0,255,0))
    M = cv2.moments(green)
    cx = (int)(M["m10"] / M["m00"])
    cy = (int)(M["m01"] / M["m00"])
    green_center = (cx,cy)

    # Uncomment following to visualize the result
    # cv2.circle(ip_image,red_center,3,(255,0,0),-1)
    # cv2.circle(ip_image,green_center,19,(255,0,0),1)
    # cv2.circle(ip_image,center,3,(255,0,0),-1)
    # cv2.line(ip_image,center,red_center,(255,0,0),2)
    # cv2.line(ip_image,center,green_center,(255,0,0),2)
    # cv2.imshow("window", ip_image)
    # cv2.waitKey(0)

    angle = getAngle(red_center,green_center,center)
    return angle




    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    line = []
    ## Reading 1 image at a time from the Images folder
    for image_name in os.listdir(images_folder_path):
        ## verifying name of image
        print(image_name)
        ## reading in image 
        ip_image = cv2.imread(images_folder_path+"/"+image_name)
        ## verifying image has content
        print(ip_image.shape)
        ## passing read in image to process function
        A = process(ip_image)
        ## saving the output in  a list variable
        line.append([str(i), image_name , str(A)])
        ## incrementing counter variable
        i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'angles.csv', 'w', newline='') as writeFile:
        writer = csv.writer(writeFile)
        writer.writerows(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
