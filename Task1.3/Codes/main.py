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
import cv2.aruco as aruco
from aruco_lib import *
import copy



########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Videos'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))
KERNEL_SIZE = 20
NOISE = 10**(-2.1)


def contrast(img, a, b):
    img = np.float32(img) * 1.2
    b = np.ones_like(img) * 10
    img = img + b
    return img.astype(np.uint8)


def edge(img):
    h, w = img.shape[:2]
    padding = cv2.copyMakeBorder(img, 21, 21, 21, 21, cv2.BORDER_WRAP)
    blur = cv2.GaussianBlur(padding, (43, 43), -1)[21:-21, 21:-21]
    Y, X = np.indices((h, w))
    distan = np.dstack([X, w-X-1, Y, h-Y-1]).min(-1)
    w = np.minimum(np.float32(distan)/21, 1.0)
    return img * w + blur * (1 - w)


def kernel_motion(d, sz=KERNEL_SIZE):
    kernel = np.ones((1, d), np.float32)
    trans = np.float32([
        [0, -1, 0],
        [1, 0, 0]
    ])
    trans[:, 2] = (sz//2, sz//2) - np.dot(trans[:, :2], ((d - 1)*0.5, 0))
    kernel = cv2.warpAffine(kernel, trans, (sz, sz), flags=cv2.INTER_CUBIC)
    return kernel


def PSF(img):
    d = 20
    psf = kernel_motion(d)
    psf /= psf.sum()
    padding = np.zeros_like(img, dtype=np.float32)
    padding[:KERNEL_SIZE, :KERNEL_SIZE] = psf
    psf = cv2.dft(padding, flags=cv2.DFT_COMPLEX_OUTPUT,
                  nonzeroRows=KERNEL_SIZE)
    return psf


def filter(img, psf):
    img = img.astype(np.float32) / 255.0
    img = edge(img)
    img = cv2.dft(img, flags=cv2.DFT_COMPLEX_OUTPUT)

    psf_sq = (psf ** 2).sum(-1)
    inv = psf / (psf_sq + NOISE)[..., np.newaxis]

    result = cv2.mulSpectrums(img, inv, 0)
    result = cv2.idft(result, flags=cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT)

    result = np.roll(result, KERNEL_SIZE // 2, 0)
    result = np.roll(result, KERNEL_SIZE // 2, 0)
    result = cv2.normalize(result,None,0,255,cv2.NORM_MINMAX)
    return result.astype(np.uint8)


def apply_filter(ip_image):
    con = contrast(ip_image, 1, 1)
    b, g, r = cv2.split(con)
    psf = PSF(b)
    b = filter(b, psf)
    g = filter(g, psf)
    r = filter(r, psf)
    return cv2.merge((b, g, r))

############################################
# Build your algorithm in this function
# ip_image: is the array of the input image
# imshow helps you view that you have loaded
# the corresponding image
############################################
def process(ip_image):
    id_list = []
    white_bg = np.ones_like(ip_image) * 255
    ip_image = ip_image[:720, :1280]
    test = apply_filter(ip_image)

    a_list = detect_Aruco(test)

    if a_list is not None and len(a_list) > 0:
        mark_Aruco(test, a_list)
        state = calculate_Robot_State(test, a_list)
        if len(state) > 0:
            tmp = state[list(state.keys())[0]]
            id_list = tmp
            white_bg[:720,:1280] = test
        cv2.imwrite(generated_folder_path+"/"+'aruco_with_id.png',white_bg)
    else:
        print("No Aruco Found")
    
    return test, id_list


    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main(val):
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(images_folder_path+"/"+"ArUco_bot.mp4")
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## getting the frame sequence
    frame_seq = int(val)*fps
    ## setting the video counter to frame sequence
    cap.set(1,frame_seq)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    ## display to see if the frame is correct
    cv2.imshow("window", frame)
    cv2.waitKey(0);
    ## calling the algorithm function
    op_image, aruco_info = process(frame)
    ## saving the output in  a list variable
    line = [str(i), "Aruco_bot.jpg" , str(aruco_info[0]), str(aruco_info[3])]
    ## incrementing counter variable
    i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'output.csv', 'w') as writeFile:
        print("About to write csv")
        writer = csv.writer(writeFile)
        writer.writerow(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main(input("time value in seconds:"))
