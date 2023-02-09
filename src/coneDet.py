import cv2
from matplotlib import pyplot as plt
import numpy as np


def draw_contour(image, contour, color=(0, 255, 0)):
    """
    Draws a contour on the provided image.
    Args:
        image: The image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour in BGR format.
    """
    cv2.drawContours(image, [contour], 0, color, 3)

def draw_multi_contours(image, contourList):
    for i in contourList:
        draw_contour(image, i)

def get_mask(image, hsv_lower, hsv_upper):
    """   
    Returns a mask containing all of the areas of image which were between hsv_lower and hsv_upper.
    
    Args:
        image: The image (stored in BGR) from which to create a mask.
        hsv_lower: The lower bound of HSV values to include in the mask.
        hsv_upper: The upper bound of HSV values to include in the mask.
    """
    # Convert hsv_lower and hsv_upper to numpy arrays so they can be used by OpenCV
    hsv_lower = np.array(hsv_lower)
    hsv_upper = np.array(hsv_upper)

    # TODO: Use the cv2.cvtColor function to switch our RGB colors to HSV colors
    new_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # TODO: Use the cv2.inRange function to highlight areas in the correct range
    mask = cv2.inRange(new_image, hsv_lower, hsv_upper)
    
    return mask

def find_contours(mask):
    """
    Returns a list of contours around all objects in a mask.
    """
    return cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

def get_largest_contour(contours, min_area=30):


    if len(contours) == 0:
        # TODO: What should we return if the list of contours is emptyy?
        return None
    maxarea = cv2.contourArea(contours[0])
    maxindex = 0
    for c in range(len(contours)):
        temparea = cv2.contourArea(contours[c])
        if(temparea > maxarea):
            maxarea = temparea
            maxindex = c
    if(maxarea < min_area):
        return None
    else:
        return contours[maxindex]

def get_large_contours(contours, min_area=1000):
    if len(contours) == 0:
            # TODO: What should we return if the list of contours is emptyy?
        return None
    maxarea = cv2.contourArea(contours[0])
    maxindex = 0
    contourList = []
    for c in range(len(contours)):
        temparea = cv2.contourArea(contours[c])
        if(temparea > 1000):
            maxarea = temparea
            # print(maxarea)
            contourList.append(contours[c])
    if(maxarea < min_area):
        return None
    else:
        # print(contourList)
        return contourList

def getCentroids(contourList):
    centers = []
    for i in contourList:
        M = cv2.moments(i)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centers.append((cX, cY))
    return centers

def drawCentroids(image, centroidList):
    for i in centroidList:
        cv2.circle(image, (i[0], i[1]), 7, (255, 255, 255), -1)

def getCenterOfCentroids(image, centroidList):
    v1 = centroidList[0]
    v2 = centroidList[1]
    xCenter = int((max(v2[0], v1[0]) - min(v2[0], v1[0]))/2 + min(v2[0], v1[0]))
    yCenter = int((max(v2[1], v1[1]) - min(v2[1], v1[1]))/2 + min(v2[1], v1[1]))
    cv2.circle(image, (xCenter, yCenter), 7, (255, 0, 0), -1)
    return ((xCenter, yCenter))

def calcDist(image, center):
    temp = image.shape
    imgCenter = np.array((temp[0]/2, temp[1]/2))
    cent = np.array(center)
    dist = cent - imgCenter
    return dist



def objectRun(tempImg):
    # print(tempImg)
    img = cv2.imread(tempImg)

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # plt.subplot(1, 1, 1)
    # plt.imshow(img_rgb)
    # plt.show()

    #red boundaries
    r1 = ((170, 100, 100), (180, 255, 255))
    r2 = ((0, 200, 200), (20, 255, 255))

    #green boundaries
    g1 = ((36, 0,0), (86, 255,255))

    mask1 = get_mask(img, r1[0], r1[1])
    mask2 = get_mask(img, r2[0], r2[1])
    greenmask = get_mask(img, g1[0], g1[1])
    mask = mask1+mask2 + greenmask
    # cv2.imshow("stop sign mask", mask)
    # cv2.waitKey(5000)
    # plt.imshow(mask)
    # plt.show()

    # Find the largest contour
    contours = find_contours(mask)
    contourList = get_large_contours(contours)

    # Draw it on the image
    image_copy = np.copy(img_rgb)
    draw_multi_contours(image_copy, contourList)
    centroids = getCentroids(contourList)
    drawCentroids(image_copy, centroids)
    center = getCenterOfCentroids(image_copy, centroids)
    # print(center)
    # print(image_copy.shape)
    dist = calcDist(image_copy, center)
    # print(dist)
    # plt.imshow(image_copy)
    # plt.show()
    return (image_copy, dist)


print(objectRun("octthird2.bmp"))