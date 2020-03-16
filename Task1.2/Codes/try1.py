    x = getRectangle(ip_image)
    cv2.rectangle(ip_image,(x[0],x[1]),(x[2],x[3]),(0,255,255),3)
    crop = ip_image[x[1]:x[3],x[0]:x[2]]
    ip_image = crop
    
    tmp = cv2.inRange(ip_image,(30,30,30),(30,30,32))
    tmp = cv2.GaussianBlur(tmp,(111,111),2,2)
    circles = cv2.HoughCircles(tmp,cv2.HOUGH_GRADIENT,1,256,
                            param1=100,param2=30,minRadius=120,maxRadius=555)
    if circles is None:
        print("fuck")
        return
    # print(circles)
    # return
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(ip_image,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(ip_image,(i[0],i[1]),2,(0,0,255),3)
