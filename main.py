import cv2
import numpy as np
import math


IMG_PATH = r"path"


def read_image_from_path(path):
    # Read the original image
    img = cv2.imread(path)
    return img


def extract_edges_from_image(img):
    """this function gets image (image type) and return new edge image"""

    # Convert to graycsale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

    # Canny Edge Detection
    edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)  # Canny Edge Detection

    # Display Canny Edge Detection Image
    #cv2.imshow('Canny Edge Detection', edges)
    #cv2.waitKey(0)

    return edges


def find_houghLinesP_ret_img(edges_image, origin_img):
    # linesP = cv2.HoughLinesP(edges_image, 1, np.pi / 180, 50, None, 50, 10) # default from opencv doc
    rho = 1
    d = 180
    threshold = 90
    minLineLength = 50
    maxLineGap = 30
    for i in range(4):
        # theta
        theta = np.pi / d
        linesP = cv2.HoughLinesP(edges_image, rho, theta, threshold, None, minLineLength, maxLineGap)

        # print pointes
        filter_linesP(linesP)
        # print pointes

        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                cv2.line(origin_img, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 1, cv2.LINE_AA)
                # draw trapeze
                #cv2.line(origin_img, (75, 480), (280, 310), (67, 23, 52), 2, cv2.LINE_AA)
                #cv2.line(origin_img, (360, 310), (565, 480), (67, 23, 52), 2, cv2.LINE_AA)
                #cv2.line(origin_img, (280, 310), (360, 310), (67, 23, 52), 2, cv2.LINE_AA)
                fontScale = 1
                cv2.putText(origin_img, f"S: ({l[0]}, {l[1]})", (l[0], l[1]), cv2.FONT_ITALIC, 0.3, 255)
                cv2.putText(origin_img, f"E: ({l[2]}, {l[3]})", (l[2], l[3]), cv2.FONT_ITALIC, 0.3, 255)

        return origin_img


def filter_linesP(lines):
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            if True:
                print(l)



if __name__ == '__main__':
    img = read_image_from_path(IMG_PATH)
    edge_img = extract_edges_from_image(img)
    houghLinesP_img = find_houghLinesP_ret_img(edge_img, img)

    cv2.imshow('Original', img)
    cv2.imshow('Edge', edge_img)
    cv2.imshow('houghLinesP', houghLinesP_img)

    cv2.waitKey(0)
