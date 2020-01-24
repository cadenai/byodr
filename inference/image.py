from functools import partial

import cv2
import numpy as np


def hwc_bgr_to_yuv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)


def hwc_to_chw(img):
    # Switch to C,H,W.
    return None if img is None else img.transpose((2, 0, 1))


def hwc_alexnet(image):
    image = cv2.resize(image, (227, 227))
    image = image.astype(np.uint8)
    return image


def caffe_dave_200_66(image, resize_wh=None, crop=(0, 0, 0, 0), dave=True, yuv=True, chw=True):
    # If resize is not the first operation, then resize the incoming image to the start of the data pipeline persistent images.
    image = image if resize_wh is None else cv2.resize(image, resize_wh)
    top, right, bottom, left = crop
    image = image[top:image.shape[0] - bottom, left:image.shape[1] - right]
    image = cv2.resize(image, (200, 66)) if dave else image
    image = hwc_bgr_to_yuv(image) if yuv else image
    image = hwc_to_chw(image) if chw else image
    image = image.astype(np.uint8)
    return image


_registered_functions = {
    'alex__227_227': partial(hwc_alexnet),
    'dave__320_240__200_66__0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(0, 0, 0, 0)),
    'dave__320_240__200_66__70_0_10_0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(70, 0, 10, 0))
}


def get_registered_function(name):
    return _registered_functions.get(name)
