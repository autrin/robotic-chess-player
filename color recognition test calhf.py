import cv2
import numpy

stream = cv2.VideoCapture(0)

while(1):
    ret, frame = stream.read()
    if not ret:
        print("Stream ended")
        break
    #convert the frame to hsv from blue green red 
    screenedframe = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #red = numpy.asarray([0,123,50])
    #ORDER IS [COLOR, SATURATION, BRIGHTNESS]
    lowcolor = 0
    highcolor = 360
    
    redLow = numpy.asarray([0,150,50])
    redHigh = numpy.asarray([255,255,255])
    greyLow = numpy.asarray([0,0,120])
    greyHigh = numpy.asarray([255,0,200])
    blueLow = numpy.asarray([98,50,50])
    blueHigh = numpy.asarray([139,255,255])

    BWredmaskedframe =cv2.inRange(screenedframe,redLow,redHigh)
    restoreColor = cv2.bitwise_and(frame,frame,mask =BWredmaskedframe)
    #onlyred = cv2.bitwise_and(frame,BWredmaskedframe,mask=None)
    cv2.imshow("Gopro",frame)
    cv2.imshow("Gopro",BWredmaskedframe)
    cv2.imshow("Gopro",restoreColor)
    
    if cv2.waitKey(1) == ord('q'):
        break


cv2.destroyAllWindows()
stream.release()