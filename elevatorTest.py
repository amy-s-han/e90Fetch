import cv2
import cv2.cv as cv
import numpy as np


img = cv2.imread('inside1.jpg',0)

# For static image
rows,cols = img.shape
M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
img = cv2.warpAffine(img,M,(cols,rows))

# print img.shape[:2]

# img = cv2.resize(img, (0, 0), fx=0.35, fy=0.35)
img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)
img = cv2.medianBlur(img,5)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

#good for 0.35 resize of phone camera picture
# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=110, param2=95)

# good for 0.3 resize of phone camera picture
# circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=90, param2=85) 

circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1.2, 20, param1=75, param2=71) 

circles = np.uint16(np.around(circles))
toSortCircles = np.int16(np.around(circles))

for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

# want to sort by y so that we can figure out the layers
sortedCircle = toSortCircles[0]
# print "before sorting", sortedCircle

sortedCircle = sortedCircle[sortedCircle[:,1].argsort()]
# print "after sorting", sortedCircle

candidates = []
numCircles = sortedCircle.shape[0]

for i in range(1, numCircles):

	# if y for this circle is far away from y of last circle,
	# that means that this is a new row of circles and this might 
	# be a candidate
	if abs(sortedCircle[i][1] - sortedCircle[i-1][1]) > 0.1 * sortedCircle[i-1][1]:
		# check if this circle has a neighbour that is on the same row.
		# if no neighbour on the same row, then add to candidates
		
		# print "\n large delta y, checking neighbours: \n"

		if abs(sortedCircle[i][1] - sortedCircle[i+1][1]) > 0.1 * sortedCircle[i-1][1]:
			candidates.append(sortedCircle[i])

	# if this is the last circle: 
	if i == numCircles - 1:
		
		#check to see if this circle is a candidate
		if len(candidates) != 0:
			if abs(sortedCircle[i][0] - candidates[0][0]) < 0.1 * candidates[0][0]:
				candidates.append(sortedCircle[i])
	
# print candidates

badIndices = []

# print "\n\n here \n\n"

# run through candidates to make sure they have similar x 
for i in range(1, len(candidates)):
	# check that the buttons are vertically aligned
	if abs(candidates[i][0] - candidates[i-1][0]) > 0.05 * candidates[i-1][0]:
		badIndices.append(i)

# print badIndices

if badIndices != []:
	for bad in badIndices:
		candidates.remove(bad)

# print candidates

for circ in candidates:
	cv2.circle(cimg, (circ[0], circ[1]), circ[2], (255, 0, 0), 3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()