import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import time as t

def getImage(path):
    """ turn jpeg into numpy array
    
    Args:
        path (string): path of the image

    Returns:
        ndarray: image
    """
    if (path.endswith(".npy")):
        return np.load(path)
    
    img = Image.open(path)
    npImg = np.asarray(img)
    
    return npImg


def createImage(img, start_angle=0, center_angle=90):
    """ create a cartesian top down image from a polar image with
    x coordinates as the distance away from the sonar and
    y coordinates as angles (in gradians)
    
    Args:
        path (string): path of the image
        start_angle (int): angle that the scan starts at
        center_angle (int): angle that the program should center the scan to

    Returns:
        ndarray: image
    """
    start = t.time()

    angle = img.shape[0]
    radius = img.shape[1]

    center = start_angle + angle // 2 # find center of the scan arc

    width = 2*radius + 1 #TODO limit size
    height = radius + 1

    polarImg = np.zeros((height, width))

    msehgrid_start = t.time()
    x_ax = np.linspace(-radius, radius, width, dtype=np.int32)
    y_ax = np.linspace(0, radius, height, dtype=np.int32)
    xx, yy = np.meshgrid(x_ax, y_ax)
    print("create meshgrid time: ", t.time() - msehgrid_start)

    theta = np.rad2deg(np.arctan2(yy, xx))*200/180
    r = np.sqrt(xx**2 + yy**2)

    np.round(theta)
    np.round(r)
    
    theta = theta.astype(np.int32)
    r = r.astype(np.int32)

    create_img_start = t.time()
    # for x in x_ax:
    #     for y in y_ax:
    for x in range(x_ax[0], x_ax[-1]+1, 2):
        for y in range(y_ax[0], y_ax[-1]+1, 3):
            theta_pt = theta[y][x + radius] - int(center_angle*200/180 - center) # shift angles to center the to center_angle
            r_pt = r[y][x + radius] # x + radius to convert x values into index values

            if (theta_pt < angle and theta_pt > 0 and r_pt < radius):
                polarImg[radius - y][radius - x] = img[theta_pt, r_pt] # radius - y flips to face the scan upward
            else:
                polarImg[radius - y][radius - x] = 0
    print("create image time: ", t.time() - create_img_start)
    print("total time: ", t.time() - start)
    return polarImg.astype(np.uint8)


def addContours(image, lower_bound=(0, 127, 0), upper_bound=(255, 255, 255), kernel=(17, 17), area_threshold=5000, line_color=(0, 0, 255)):
    """ gaussian blurs the image then adds contours
    
    Args:
        image (ndarray): image to process
        lower_bound (tuple): lower bound when thresholding the image to find contour regions
        upper_bound (tuple): upper bound when thresholding the image to find contour regions
        kernel (tuple): kernel to use when applying gaussian blur
        area_threshold (int or float): only shows contours that are bigger than this value
        line_color (tuple): BGR value for cv2 to draw contours and centroid dot
    """
    mask = cv2.inRange(image, lower_bound, upper_bound)

    blurred_img = cv2.GaussianBlur(mask, kernel, 0)

    contours, hierarchy = cv2.findContours(blurred_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shapes = []
    print("Num contours", len(contours))
    for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        if moments['m00'] > area_threshold:
            shapes.append(contours[i])
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv2.circle(image, (cx, cy), 8, line_color, -1)
            cv2.drawContours(image, contours, i, line_color, 2)
    printCirularity(shapes)
    return image

def printCirularity(contours):

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        if perimeter == 0:
            break
        circularity = 4*np.pi*(area/perimeter*perimeter)
        print("Circularity: ", circularity)


def main():
    start = t.time()

    sonar_img = np.load('onboard/catkin_ws/src/sonar/sampleData/sonar_sweep_1.npy')
    # print(sonar_img)

    sonar_img_polar = createImage(sonar_img)
    sonar_img_polar = cv2.cvtColor(sonar_img_polar.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    sonar_img_polar = cv2.applyColorMap(sonar_img_polar, cv2.COLORMAP_VIRIDIS)
    addContours(sonar_img_polar)
    resized_img = cv2.resize(sonar_img_polar, (sonar_img_polar.shape[1] // 2, sonar_img_polar.shape[0] // 2))
    
    # print(sonar_img_polar.shape)

    cv2.imshow('sonar image', resized_img)
    print(t.time() - start)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
   

if __name__ == '__main__':
    main()