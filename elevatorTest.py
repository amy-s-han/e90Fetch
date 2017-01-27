import cv2
import cv2.cv as cv
import numpy as np




img = cv2.imread('inside1.jpg',0)
img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
img = cv2.medianBlur(img,5)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

print "here"
# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1, 20,
#                             param1=50,param2=30,minRadius=0,maxRadius=0)

# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)
circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=80)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()