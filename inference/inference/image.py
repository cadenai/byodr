from __future__ import absolute_import
from functools import partial

import cv2
import numpy as np

from byodr.utils.option import PropertyError


def hwc_bgr_to_yuv(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)


def hwc_to_chw(img):
    # Switch to C,H,W.
    return None if img is None else img.transpose((2, 0, 1))


def hwc_alexnet(image, resize_wh=None):
    image = image if resize_wh is None else cv2.resize(image, resize_wh)
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


def _exr_dave_img(image):
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    frame[:190, :, :] = image[50:240, :, :]
    frame[190:, :, :] = image[-190:-140, :, :]
    return caffe_dave_200_66(frame)


def _exr_alex_img(image, top=True):
    return hwc_alexnet(image[:240] if top else image[-240:])


class Alternator(object):
    def __init__(self, f1, f2):
        self.f_list = [f1, f2]
        self.index = 0

    def __call__(self, *args, **kwargs):
        self.index = 0 if self.index == 1 else 1
        return self.f_list[self.index](*args, **kwargs)


_registered_functions = {
    'alex__227_227': partial(hwc_alexnet, resize_wh=(320, 240)),
    'dave__320_240__200_66__0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(0, 0, 0, 0)),
    'dave__320_240__200_66__70_0_10_0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(70, 0, 10, 0)),
    'dave__exr1': partial(_exr_dave_img),
    'alex__exr1': Alternator(partial(_exr_alex_img, top=True), partial(_exr_alex_img, top=False))
}


def get_registered_function(key, errors, **kwargs):
    name = kwargs.get(key, None)
    if name in _registered_functions:
        return _registered_functions.get(name)
    else:
        errors.append(PropertyError(key=key, msg='Not a function'))
        return lambda x: x
