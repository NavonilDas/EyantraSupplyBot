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




############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    ## placeholder image
    sector_image = np.ones(ip_image.shape[:2],np.uint8)*255
    ## check value is white or not
    print(sector_image[0,0])

    # tmp = ip_image.copy()
    ip_image = cv2.inRange(ip_image,(250,250,250),(255,255,255))
    h,w = ip_image.shape[:2]
    
    mask = np.zeros((h+2,w+2),np.uint8)
    cv2.floodFill(ip_image,mask,(0,0),(255,255,255))
    ip_image = np.bitwise_not(ip_image)
    
    mask[:,:] = 0
    cv2.floodFill(ip_image,mask,(0,0),(255,255,255))
    
    edged = cv2.Canny(ip_image, 30, 200)
    contours, hierarchy = cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    
    tc = []
    for c in contours:
        if cv2.pointPolygonTest(c,(w//2,h//2),True) < 0:
            tc.append(c)
    del contours

    black = (30,30,32)
    cv2.fillConvexPoly(sector_image,np.array(tc[0]),black)
    # cv2.drawContours(sector_image,tc,-1,(30,30,32),2)

    # cv2.fillConvexPoly(tmp,np.array(tc[0]),black)
    cv2.imshow("window", sector_image)
    cv2.waitKey(0);

    return sector_image




    
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
    ## Reading 1 image at a time from the Images folder
    for image_name in os.listdir(images_folder_path):
        ## verifying name of image
        print(image_name)
        ## reading in image 
        ip_image = cv2.imread(images_folder_path+"/"+image_name)
        ## verifying image has content
        print(ip_image.shape)
        ## passing read in image to process function
        sector_image = process(ip_image)
        ## saving the output in  an image of said name in the Generated folder
        cv2.imwrite(generated_folder_path+"/"+image_name[0:len(image_name)-4]+"_fill_in.png", sector_image)
        i+=1


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
