def getRectangle(ip_image):
    tx,ty = -1,-1
    bx,by = -1,-1
    h,w,_ = ip_image.shape
    for i in range(h):
        if (ip_image[i,int(w/2)] == [204,0,153]).all():
            ty = i
        if ty != -1 and (ip_image[i,int(w/2)] != [204,0,153]).all():
            break
    
    for i in range(h-1,0,-1):
        if (ip_image[i,int(w/2)] == [204,0,153]).all():
            by = i
        if by != -1 and (ip_image[i,int(w/2)] != [204,0,153]).all():
            break

    for i in range(w):
        if (ip_image[int(h/2),i] == [204,0,153]).all():
            tx = i
        if tx != -1 and (ip_image[int(h/2),i] != [204,0,153]).all():
            break
    
    for i in range(w-1,0,-1):
        if (ip_image[int(h/2),i] == [204,0,153]).all():
            bx = i
        if bx !=-1 and (ip_image[int(h/2),i] != [204,0,153]).all():
            bx = i
            break

    if bx == -1 or by == -1 or tx == -1 or ty == -1:
        return []
    return [tx,ty,bx,by]





def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result