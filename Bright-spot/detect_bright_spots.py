# USAGE
# python detect_bright_spots.py --image images/lights_01.png

# import the necessary packages
#from imutils import contours
#from skimage import measure
import numpy as np
#import argparse
#import imutils
import cv2



# load the image, convert it to grayscale, and blur it
image = cv2.imread("lights_01.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)
cv2.imshow("blurred", blurred)
# threshold the image to reveal light regions in the
# blurred image
thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)[1]
cv2.imshow("thresh", thresh)
 
im2, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
boxes = []
for c in cnts:
    (x, y, w, h) = cv2.boundingRect(c)
    boxes.append([x,y, x+w,y+h])

boxes = np.asarray(boxes)
# need an extra "min/max" for contours outside the frame
left = np.min(boxes[:,0])
top = np.min(boxes[:,1])
right = np.max(boxes[:,2])
bottom = np.max(boxes[:,3])

cv2.rectangle(image, (left,top), (right,bottom), (255, 0, 0), 2)

#cv2.drawContours(image, contours, -1, (255,0,0), 2)
print ("number of bright spots are" , len(cnts))

# show the output image
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
