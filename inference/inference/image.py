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


def _image(image, im_width=32, im_height=32, crop_top=0, crop_bottom=0, yuv=False, chw=False, dtype=np.uint8):
    image = cv2.resize(image, (im_width, im_height + crop_top + crop_bottom))[crop_top: im_height + crop_top, ...]
    image = hwc_bgr_to_yuv(image) if yuv else image
    image = image.astype(dtype)
    image = hwc_to_chw(image) if chw else image
    return image


def hwc_alexnet(image, resize_wh=None):
    image = image if resize_wh is None else cv2.resize(image, resize_wh)
    return _image(image, im_width=227, im_height=227, yuv=False, chw=False)


def hwc_squeeze(image, resize_wh=None):
    image = image if resize_wh is None else cv2.resize(image, resize_wh)
    return _image(image, im_width=200, im_height=100, yuv=False, chw=False)


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


class Alternator(object):
    def __init__(self, f1, f2):
        self.f_list = [f1, f2]
        self.index = 0

    def __call__(self, *args, **kwargs):
        self.index = 0 if self.index == 1 else 1
        return self.f_list[self.index](*args, **kwargs)


_registered_functions = {
    'alex__200_100': partial(hwc_squeeze, resize_wh=(320, 240)),
    'dave__320_240__200_66__0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(0, 0, 0, 0), yuv=False, chw=False),
    'dave__320_240__200_66__70_0_10_0': partial(caffe_dave_200_66, resize_wh=(320, 240), crop=(70, 0, 10, 0), yuv=False, chw=False)
    # 'alex__exr1': Alternator(partial(_exr_alex_img, top=True), partial(_exr_alex_img, top=False))
}


def get_registered_function(key, default_value, errors, **kwargs):
    name = kwargs.get(key, default_value)
    if name in _registered_functions:
        return _registered_functions.get(name)
    else:
        errors.append(PropertyError(key=key, msg='Not a function'))
        return lambda x: x
