# ----- Sentry Program for Science fair project -----------
# Program made by: Marco on 28/02/19

# DISCLAIMER THE OBJECT TRAINING WAS PROVIDED BY Tensorflow detection model zoo
# pulled from here: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md


# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import argparse
import sys

from utils import label_map_util
from utils import visualization_utils as vis_util

# Raspberry Pi modules
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera

# Cam resolution dimensions
WIDTH = 1280
HEIGHT = 720

# PINs
    # Analog Pins
RIGHT_PWM = 25
LEFT_PWM = 24
    # Digital Pins
RIGHT_DIR = 23
LEFT_DIR = 18
# --------------------------------
# Setting pin settings
GPIO.setup(RIGHT_PWM,GPIO.OUT)
GPIO.setup(LEFT_PWM,GPIO.OUT)
GPIO.setup(LEFT_DIR,GPIO.OUT)
GPIO.setup(RIGHT_DIR,GPIO.OUT)
# --------------------------

cam_type = "picamera"

MODEL_NAME = "facessd_mobilenet_v2_quantized_320x320_open_imagev4"
