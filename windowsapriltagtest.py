import cv2
import numpy as np
import pupil_apriltags as apriltag

stream = cv2.VideoCapture(0)


while(1):
    ret, camera = stream.read()
    
    if not ret:
        print("Stream ended")
        break

    #convert the frame to greyscale
    screenedcamera = cv2.cvtColor(camera, cv2.COLOR_BGR2GRAY)
    aprildetector = apriltag.Detector()
    tags = aprildetector.detect(screenedcamera)

    # for tag in tags:
    #     (ptA,ptB,ptC,ptD)= tag.corners
        
    #     cv2.line(camera,ptA,ptB,(0,255,0),2)
    #     cv2.line(camera,ptB,ptC,(0,255,0),2)
    #     cv2.line(camera,ptC,ptD,(0,255,0),2)
    #     cv2.line(camera,ptD,ptA,(0,255,0),2)

    #     (cX,cY) = (int(tag.center[0]),int(tag.center[1]))
    #     cv2.circle(camera,(cX,cY),5,(0,0,255),-1)
    
    if cv2.waitKey(1) == ord('q'):
        break
    cv2.imshow("Gopro",camera)
    

cv2.destroyAllWindows()
stream.release()