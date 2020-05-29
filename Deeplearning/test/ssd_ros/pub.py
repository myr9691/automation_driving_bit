#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import tflite_runtime.interpreter as tflite
import numpy as np
import re

import cv2

import pyrealsense2 as rs

import time

class ROS:

    def __init__(self):

        # Pub Raw Image
        self.raw_image_pub = rospy.Publisher('camera/raw_image', Image, queue_size=1)
        self.bridge = CvBridge()

        # Pub Compressed Image
        self.compressed_image_pub = rospy.Publisher('camera/compressed_image', CompressedImage, queue_size=1)
        # We do not use cv_bridge it does not support CompressedImage in python

    def pub_raw_image(self, image):

        # Create Image msg
        try:
            raw_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        rospy.loginfo("Publish the Raw Image")
        self.raw_image_pub.publish(raw_image_msg)

    def pub_compressed_image(self, image):

        # Create CompressedImage
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # Publish Compressed Image
        rospy.loginfo("Publish the Compressed Image")
        self.compressed_image_pub.publish(compressed_image_msg)


class Realsense:

    def __init__(self):

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def get_frames(self):

        frames = self.pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        return color_frame
        
    def stop(self):

        self.pipeline.stop()

class TFLite:

    def __init__(self):

        # Load COCO label map
        self.labelmap_dir = '/home/michael/TensorFlow/TFLite/models/ssd_coco/labelmap.txt'
        self.labels = self.load_labels(self.labelmap_dir)

        # Load TFLite model and allocate tensors.
        self.ssd_v2_edgetpu_detect_tflite_dir = '/home/michael/TensorFlow/TFLite/models/ssd_coco/detect_edgetpu.tflite'
        self.interpreter = tflite.Interpreter(model_path=self.ssd_v2_edgetpu_detect_tflite_dir, experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
        self.interpreter.allocate_tensors()

    def load_labels(self, path):
        """Loads the labels file. Supports files with or without index numbers."""
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    def get_output_tensor(self, interpreter, index):
        """Returns the output tensor at the given index."""
        output_details = interpreter.get_output_details()[index]
        tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
        return tensor

    def detect_objects(self, interpreter, image, threshold):
        """Returns a list of detection results, each a dictionary of object info."""
        input_details = interpreter.get_input_details()
        interpreter.set_tensor(input_details[0]['index'], image)
        interpreter.invoke()

        # Get all output details
        boxes = self.get_output_tensor(interpreter, 0)
        classes = self.get_output_tensor(interpreter, 1)
        scores = self.get_output_tensor(interpreter, 2)
        count = int(self.get_output_tensor(interpreter, 3))

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i]
                }
                results.append(result)
        return results

    def show_inference_image(self, image, results, labels):
        result_size = len(results)
        for idx, obj in enumerate(results):
            # print(obj)
            # Prepare image width, height
            im_height, im_width, _ = image.shape
        
            # Prepare score, name
            score = obj['score']
            name = labels[obj['class_id'] + 1]
            # print('idx: {}, name: {}'.format(idx, name))
        
            # Prepare boundary box
            ymin, xmin, ymax, xmax = obj['bounding_box']
            xmin = int(xmin * im_width)
            xmax = int(xmax * im_width)
            ymin = int(ymin * im_height)
            ymax = int(ymax * im_height)

            # Draw rectangle to desired thickness
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        
            # Annotate image with label and confidence score
            cv2.putText(image, '{}: {:.2f}'.format(name, score), (xmin, ymin + 12), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
    
        return image

    def inference(self, image):
        
        # Expand Dimensions for Inference and resize 300*300
        tflite_image = cv2.resize(image, dsize=(300, 300), interpolation=cv2.INTER_AREA)
        tflite_image = np.expand_dims(tflite_image, 0)

        # Detection
        threshold = 0.5
        results = self.detect_objects(self.interpreter, tflite_image, threshold)

        # Return result image
        result_image = self.show_inference_image(image, results, self.labels)
        
        return result_image
        

def main():
    ros = ROS()
    tf = TFLite()
    camera = Realsense()

    rospy.init_node('image_pub', anonymous=True)

    while not rospy.is_shutdown():

        start_time = time.time()

        # Get the Image from Realsense Camera
        color_frame = camera.get_frames()
        #if not depth_frame or not color_frame:
        if not color_frame:
            continue

        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Show Color Image
        #cv2.imshow('Color Image', color_image)

        # Inference Image
        inference_image = tf.inference(color_image)

        # Show Inference Image
        cv2.imshow('Inference Image', inference_image)
        
        # Publish the Image
        ros.pub_raw_image(inference_image)
        #ros.pub_compressed_image(inference_image)

        end_time = time.time()
        print('FPS = {0:03d}'.format(int(1/(end_time - start_time))), end='\r')

        cv2.waitKey(1)

    cv2.destroyAllWindows()
    camera.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
