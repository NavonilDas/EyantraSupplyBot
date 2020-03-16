import cv2
import cv2.aruco as aruco
import math
import serial
import numpy as np
import time

# Documentation in format pep257


############### Change Values to get better result

# Global MAXimum and minimum colour for coin detection
RED_MAX = (179, 255, 255)
RED_MIN = (170, 100, 20)
GREEN_MAX = (65,255,255)
GREEN_MIN = (33,100,20)
ORANGE_MAX = (6,255,255)
ORANGE_MIN = (0,100,20)

# Radius of the aurco to ignore red colour from the bot
AURCO_RANGE = 50

# Radius for the orange Region
ORANGE_RADIUS = 70

# Radius for green colour to be ignored
GREEN_RADIUS = 115

BIAS = -10

# For Node Detection
NODES_ANGLE = [
    [328,20],
    [20,49],
    [49,92],
    [92,138],
    [138,166],
    [166,191],
    [191,240],
    [240,272],
    [272,328]
]


# Global variable to store the node of aurco
initial_node = -1

morph_square = np.ones((5,5),np.uint8)
morph_circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# Opening Serial Port on computer
xbee = None
try:
    print("Trying to connect to XBEE")
    xbee = serial.Serial('COM7',115200)
    # Wait for the connection
    time.sleep(5)
    print("Connected")
except:
    print("Error While Opening Port")

def test_nodes(image):
    pass



def send(data):
    """This function send data to XBEE and waits till it get response(Acknowledgement)"""
    if not (xbee is None) and xbee.is_open:
        xbee.write(data)
        print("Waiting For Response....")
        ch = xbee.read()
    else:
        print("XBEE not connected")          

def brightness_contrast(image, brightness, contrast):
    """This functions changes brighness and contrast of an image having contrast in range of 0 to 100"""
    return cv2.addWeighted(image,1 + float(contrast) / 100.,image,0,float(brightness))

def detect_aruco(image):
    """Detects aurco and returns center and one side point"""
    # image = brightness_contrast(image,0,10)

    # Convert image to Grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #creating aruco_dict with 5x5 bits with max 250 ids..so ids ranges from 0-249
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    # Parameters for the detectMarker process
    parameters = aruco.DetectorParameters_create()
    
    # extract the corners and id
    corners, ids , _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    # As we know there is only one aurco so we don't need to iterate
    if len(corners) >  0 and len(corners[0]) > 0:
        # Choose the sides and id of first aurco
        sides = corners[0][0]
        cid = ids[0][0]
        
        # Calculate the centroid of a quadilateral
        center = sides[0] + sides[1] + sides[2] + sides[3]
        center[:] = [int(x//4) for x in center]

        # Draw a Circle to specify the position of aurco
        cv2.circle(image,tuple(center),AURCO_RANGE,(0,255,0),2)
        # Print the id of aurco
        cv2.putText(image,str(cid),tuple(center),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        cv2.imshow("aruco",image)
        return (tuple(center),tuple(sides[0]))
    else:
        print("No Aruco Found")
        return None,None

def get_angle(a,b,o):
    """This function returns angle between three points a,b,o or <aob within the range of 0-180
    Keyword arguments:
    a -- tuple (x,y)
    b -- tuple (x,y)
    c -- tuple (x,y)
    """
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

def crop_arena(image):
    """This function is to remove Eyantra LOGO as it is also red in color and causes trouble.
    Returns a bounding rectangle

    Keyword arguments:
    image -- opencv image
    """
    tmp = image
    # Convert image to grayscale and find the threshold
    tmp = cv2.cvtColor(tmp,cv2.COLOR_BGR2GRAY)
    _,tmp = cv2.threshold(tmp,100,255,cv2.THRESH_BINARY)

    # detect edge and countour
    edges = cv2.Canny(tmp,100,200)
    countours,_ = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Choose the countour having the maximum area
    max_area,max_ind = 0,0
    for i in range(0,len(countours)):
        x,y,w,h = cv2.boundingRect(countours[i])
        area = w*h
        if max_area < area:
            max_area = area
            max_ind = i
    return cv2.boundingRect(countours[max_ind])

def detect_orange(image):
    """Returns the center of orange ie. center of flex
    Keyword arguments:
    image -- numpy array
    """
    # Create a mask of orange colour
    tmp = cv2.inRange(image,ORANGE_MIN,ORANGE_MAX)
    # Find contours 
    countours,_ = cv2.findContours(tmp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # choose the contour having maximum area
    max_area,max_ind = 0,0
    for i in range(0,len(countours)):
        area = cv2.contourArea(countours[i])
        if max_area < area:
            max_area = area
            max_ind = i
    
    # Caclulate center using moments
    # https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
    M = cv2.moments(countours[max_ind])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # return the center
    return (cX,cY)

def get_radius(point_a,point_b):
    """Returns Euclidean distance between point_a,point_b
    Keyword arguments:
    point_a -- tuple (x,y)
    point_b -- tuple (x,y)
    """
    X = point_a[0] - point_b[0]
    Y = point_a[1] - point_b[1]
    return round(math.sqrt(X*X + Y*Y))

def inside_circle(point,center,radius):
    """Return true if point lies inside circle 
    Keyword arguments:
    point -- tuple (x,y)
    center -- tuple (x,y) center of circle
    radius -- tuple (x,y) radius of circle 
    """
    dx = (point[0] - center[0])
    dy = (point[1] - center[1])
    return ((dx*dx) + (dy*dy) <= radius * radius)

def detect_red(image,aruco_center):
    """Detect Red coin on the image and returns list of centers
    Keyword arguments:
    image -- numpy array
    aruco_center -- tuple (x,y) denoting the center of aruco
    """
    # Add Blur to remove noise or to smooth image
    image = cv2.medianBlur(image,5)
    # Get Red Color mask
    remove_ref = cv2.inRange(image,RED_MIN,RED_MAX)
    
    # Perform Opening (Image Morphing) to remove noise inside the image
    # Opening -> Erosion then dilation 
    erode = cv2.erode(remove_ref,morph_square,iterations=1)
    dilate = cv2.dilate(erode,morph_circle,iterations=1)

    # Find all the colour detected
    cntrs,_ = cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Check if Countours are present
    # TODO: if orange colour is causing trouble then check if countour is not inside orange 
    if len(cntrs) > 0:
        centers = []
        for countour in cntrs:
            x,y,w,h = cv2.boundingRect(countour)
            tmp_center = (x+w//2,y+h//2)

            # choose countour which are not inside the bot
            if not (aruco_center is None) and inside_circle(tmp_center,aruco_center,AURCO_RANGE):
                continue
            else:
                centers.append(tmp_center)
            # Return the center of red coin
        return centers
    else:
        print("Error in detecting red coin")
        return []


def detect_green(image,orange_center):
    """Returns list of green coin centers

    Keyword arguments:
    image -- numpy array
    orange_center -- tuple (x,y) center of the orange
    """
    # Blur image to remove noise
    image = cv2.medianBlur(image,5)
    
    # Extract the green mask
    tmp = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,tmp = cv2.threshold(tmp,110,255,cv2.THRESH_BINARY)
    green = cv2.inRange(image,GREEN_MIN,GREEN_MAX)
    remove_ref = cv2.bitwise_and(green,tmp)

    # Perform Opening (Image Morphing) to remove noise inside the image
    # Opening -> Erosion then dilation 
    erode = cv2.erode(remove_ref,morph_square,iterations=1)
    dilate = cv2.dilate(erode,morph_circle,iterations=1)
    # find countours
    cntrs,_ = cv2.findContours(dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # choose those green which are ouside the dead zone
    greens = []    
    for cnt in cntrs:
        x,y,w,h = cv2.boundingRect(cnt)
        tmpcenter = (int(x+w/2),int(y+h/2))
        if not inside_circle(tmpcenter,orange_center,GREEN_RADIUS):
            greens.append(tmpcenter)
    
    # return center of green colours
    return greens



def process_coins(image):
    """Returns the center of coin,center of flex and aruco center
    Keyword arguments:
    image -- numpy array
    """
    orig = image.copy()
    # Add Contast to image as the image is a little dark
    image = brightness_contrast(image,0,45)
    
    # Add Blur to remove Noise
    image = cv2.medianBlur(image,5)
    # Call Crop Arena function to get bounds for the required image
    x,y,w,h = crop_arena(image)
    # Crop Image
    image = image[y:y+h,x:x+w]
    
    # Find the position of aruco
    aruco_center,_ = detect_aruco(orig[y:y+h,x:x+w])
    if aruco_center is None:
        return

    # Convert rgb image to hsv
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # Find the center of flex
    orange_center = detect_orange(hsv_img)

    # Find the centers of all red coins
    reds = []
    reds = detect_red(hsv_img,aruco_center)

    # Find the centers of all green coins
    greens = []
    greens = detect_green(hsv_img,orange_center)
    
    return (orange_center,reds,greens,aruco_center)

def get_node_no(deg):
    """Returns the index of Node on the basis NODES_ANGLE table
    Keyword arguments:
    deg -- Degrees
    """
    # if deg is less than 20 then it is the first index
    aur_node = 0
    if deg > 20:
        for node in NODES_ANGLE:
            if node[0] <= deg and node[1] >= deg:
                break
            aur_node = aur_node + 1
    return aur_node

def get_360_angle(point_a,point_b):
    """Return angle between point_a and point_b using (0,0) as reference point
    the angle is in range of 0-360
    Keyword arguments:
    point_a -- tuple (x,y)
    point_b -- tuple (x,y)
    """
    rad = math.atan2(point_b[1] - point_a[1],point_b[0] - point_a[0])
    deg = math.degrees(rad)
    if deg < 0:
        deg = 360 + deg
    deg = round(deg)
    return deg

def process_nodes(orange_center,reds,greens,aruco_center):
    """Print Position of coins(Aids)
    Keyword arguments:
    orange_center -- tuple (x,y) center position of the orange colour
    reds -- list containing center of red coins
    greens -- list containing center of green coins
    aruco_center -- tuple (x,y) center of aruco marker
    """
    global initial_node
    if (aruco_center is None):
        print("Error No aurco can't process node")
        return
    deg = get_360_angle(orange_center,aruco_center)
    aur = get_node_no(deg)
    initial_node = aur

    # Print the node number of medical aids
    for red in reds:
        tmp = get_node_no(get_360_angle(orange_center,red))
        print("Medical Aid at {}".format(tmp - aur + 1))
    
    # Print the node number of relief aids    
    for green in greens:
        tmp = get_node_no(get_360_angle(orange_center,green))
        print("Relief Aid at {}".format(tmp - aur + 1))
    
def start(frame,reds,greens,orange_center):
    """Returns True if all coins are processed else return new list of red and green coins"""
    global initial_node
    # get the position of aruco marker
    aruco_center,_ = detect_aruco(frame)

    if aruco_center is None:
        return None,None
    
    temp = math.atan2(orange_center[1] - aruco_center[1],orange_center[0] - aruco_center[0])
    temp = round(math.degrees(temp))

    # get the node number of the bot
    x = get_node_no(get_360_angle(orange_center,aruco_center))

    # check if all coins are processed and bot is at the initial angle
    if (len(reds) == 0 and len(greens) == 0 and x == initial_node):
        print("Done")
        send(b'e')
        return True,None

    # Check if the Bot is near to the red coin having a bias
    remove_from_red = -1
    for i in range(len(reds)):
        red = reds[i]
        ang = math.atan2(orange_center[1] - red[1],orange_center[0] - red[0])
        ang = round(math.degrees(ang))
        if (temp - ang) == BIAS:
            # Select the coin
            remove_from_red = i
            # Send command to bot to service the coin
            send(b'h')
    
    # The coin which is serviced need to be removed
    if remove_from_red != -1:
        del reds[remove_from_red]

    # Check if the Bot is near to the green coin having a bias
    remove_from_green = -1
    for i in range(len(greens)):
        green = greens[i]
        ang = math.atan2(orange_center[1] - green[1],orange_center[0] - green[0])
        ang = round(math.degrees(ang))
        
        if (temp - ang) == BIAS:
            # Select the coin
            remove_from_green = i
            # Send command to bot to service the coin
            send(b'h')
    
    # The coin which is serviced need to be removed
    if remove_from_green != -1:
        del greens[remove_from_green]
    # return new list of coins
    return reds,greens



def main():
    # Open Camera
    cap = cv2.VideoCapture(0)
    # Get the FPS
    fps = cap.get(cv2.CAP_PROP_FPS)
    # set size to 640x480 
    cap.set(3, 640)
    cap.set(4, 480)

    # a frame counter
    timer = 0

    # variable to store red coins,green coins and center of the orange
    orange_center,reds,green,aruco_center = None,None,None,None
    
    # Variable to store frame and ret
    ret,frame = True,None

    # Calculate the wait time using the fps
    waitTime = int(1000/fps)

    # loop till camera doesn't return a frame
    while(ret):
        # read the frame
        ret, frame = cap.read()
        # if no frame is found break
        if not ret:
            break

        # Show the frame
        cv2.imshow("frame",frame)
        
        if timer < 10:
            # Calculate centers upto 10 frames as till this video stabilize
            orange_center,reds,green,aruco_center = process_coins(frame)
            # Print the Number of coins
            if len(reds) > 0 or len(green) > 0:
                total = len(reds) + len(green)
                print("No of detected coins {}".format(total))
        elif timer < 12:
            # Print the nodes
            process_nodes(orange_center,reds,green,aruco_center)
        elif timer < 13:
            # Send signal to start the bot
            send(b's')
        else:
            # Start Processing
            tmpreds,tmpgreens = start(frame,reds,green,orange_center)

            # Check if the value return is boolean or list
            if isinstance(tmpreds,bool) and tmpreds:
                break
            elif isinstance(tmpreds,list):
                # if value is a list update the new list of red and green coins center
                reds,green = tmpreds,tmpgreens
            else:
                pass
        
        # press x to quit
        if cv2.waitKey(waitTime) == ord('x'):
            break
        # increment the timer 
        timer = timer + 1
    # Release the camera 
    cap.release()

try:
    main()
except Exception as e:
    print("Something went wrong")
    print(e)

# Destroy/Close all windows opened by openc
cv2.destroyAllWindows()

# close serial port
if not(xbee is None) and xbee.is_open:
    xbee.close()