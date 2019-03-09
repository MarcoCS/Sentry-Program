######## Picamera Sentry Program #########
# Author: Marco Sin
# Date 2019 March 7th

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py
## Borrowed and modified Evan Juras' Code and tutorial from here:
## https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi/blob/master/Pet_detector.py


# Import modules
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
from time import sleep
import serial
# Pi Packages
import RPi.GPIO as GPIO


# Set up camera constants
IM_WIDTH = 1280
IM_HEIGHT = 720

# Motor names and pins
right = 12
left = 11
shoot = 13
GPIO.setmode(GPIO.BOARD)
GPIO.setup(right, GPIO.OUT)
GPIO.setup(left, GPIO.OUT)
GPIO.setup(shoot, GPIO.OUT)


GPIO.setwarnings(False)


# Setting up Pins
GPIO.setup(right, GPIO.OUT)
GPIO.setup(left, GPIO.OUT)
GPIO.setup(shoot, GPIO.OUT)




target_det = "False"

# a USB webcam will be used)
camera_type = 'picamera'
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module
MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

# Grab path to current working directory
CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb') # I don't quite understand this enough
PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')
NUM_CLASSES = 90
# -----------

## Load the label map.
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

#Establish Variables
global x
x = 0
global target_status
target_status = ""
# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)



# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX



if camera_type == 'picamera':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 20
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)

    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        global x
        global target_status

        t1 = cv2.getTickCount()
        
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})
        cv2.rectangle(frame,(int(440),int(710)),(int(840),int(10)),(20,20,255),3)

        # Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.40)
        if (int(classes[0][0]) == 1):
            x = int(((boxes[0][0][1]+boxes[0][0][3])/2)*IM_WIDTH)
            print(x)
        # 21 pixels per degree of rotation and measuring from distance in the middle of the screen
        degrees_from_target = (x - 640) / 21
        # Formula: distance_from_center_pixels / 21 = degrees of turning required
        
        # we figure percent of rotation we need
        rotation_required = degrees_from_target / 360
        # -------------------------------
    
        # Finding how long we need to rotate at full speed
        rotation_time = abs(rotation_required) * 1.35
        # Should return how long we should rotate in seconds.
        
        
            
        if 440 < x < 840:   # If within this range it is considered centered and maybe a hit
            target_status = "Target centered"
            GPIO.output(shoot, GPIO.HIGH)
            GPIO.output(shoot, GPIO.LOW)
        elif 0 < x < 440:    
            target_status = "Move left"
            GPIO.output(left, GPIO.HIGH)
            sleep(rotation_time)
            GPIO.output(shoot, GPIO.HIGH)
            GPIO.output(shoot, GPIO.LOW)
            GPIO.output(left, GPIO.LOW)
        elif x > 840:
            target_status = "Move Right"
            GPIO.output(right, GPIO.HIGH)
            sleep(rotation_time)
            GPIO.output(shoot, GPIO.HIGH)
            GPIO.output(shoot, GPIO.LOW)
            GPIO.output(right, GPIO.LOW)
            
        # Program for some reason defaults to outputting 0 if it doesn't see anything, I don't know why.
        if x == 0:
            target_status = "None Detected"
            degrees_from_target = "N/A"
        
        
        # Drawing frame to canvas
        try:
            cv2.putText(frame,"FPS: {0:.2f}  {1} {2:.2f} degrees".format(frame_rate_calc, target_status, degrees_from_target),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
        except: 
            cv2.putText(frame,"FPS: {0:.2f}  {1} {2} degrees".format(frame_rate_calc, target_status, degrees_from_target),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
        cv2.imshow('Sentry Program', frame)
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc = 1/time1
        # -------
        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)

    camera.close()

GPIO.cleanup()
cv2.destroyAllWindows()

