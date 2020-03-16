    pup = cv2.inRange(ip_image,(200,0,152),(204,0,153))
    skyblue = cv2.inRange(ip_image,(250,153,102),(255,153,102))
    tyyy = skyblue.copy()
    # skyblue = np.concatenate((pup,skyblue),axis=0)
    skyblue = cv2.addWeighted(pup,1.0,skyblue,1.0,0)
    skyblue = cv2.GaussianBlur(skyblue,(111,111),2,2)

    circles = cv2.HoughCircles(skyblue,cv2.HOUGH_GRADIENT,1,256,
                            param1=100,param2=30,minRadius=280,maxRadius=340)
    if circles is None:
        print("fuck")
        return

    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(ip_image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(ip_image,(i[0],i[1]),2,(0,0,255),3)

    circles = cv2.HoughCircles(skyblue,cv2.HOUGH_GRADIENT,1,256,
                            param1=100,param2=30,minRadius=200,maxRadius=300)
    if circles is None:
        print("fuck")
        return
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(ip_image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(ip_image,(i[0],i[1]),2,(0,0,255),3)


