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




if __name__ == '__main__':
    img = read_image_from_path(IMG_PATH)
    edge_img = extract_edges_from_image(img)
    cv2.imshow('Original', img)
    cv2.imshow('Edge', edge_img)
    cv2.waitKey(0)
