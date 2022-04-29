#!/usr/bin/env python


import tensorflow as tf
import keras_preprocessing
import keras
import numpy as np
from keras_preprocessing import image
from keras.models import load_model
import rospy
import sys
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Image
from apriltags2to1.msg import AprilTagDetection, AprilTagDetectionArray
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import socket


class Classifier:
  def __init__(self, robot_name):
    """ initialize image processing, tensorflow, and ROS parameters required by this node """

    # initialize image processing object
    self.bridge = CvBridge()

    # initialize variables with rosparam arguments
    self.robot_name = robot_name
    self.model_file_path = rospy.get_param('/TENSORFLOW/model_file_path')

    # load trained model for classification
    self.model = tf.keras.models.load_model(self.model_file_path)

    # setup required ROS publishers and subscribers
    self.image_data_subscriber = message_filters.Subscriber('camera/image', Image)
    self.apriltag_subscriber = message_filters.Subscriber('targets', AprilTagDetectionArray)

    # use an approximate time filter to keep the above two subscribers synchronized to a single callback
    self.synchronizer = message_filters.ApproximateTimeSynchronizer(
      [self.image_data_subscriber, self.apriltag_subscriber], 10, 0.1, allow_headerless=True)
    self.synchronizer.registerCallback(self.image_data_callback)
    self.int_classifier_publisher = rospy.Publisher('classifier_int', UInt8, queue_size=5)
    self.classifier_publisher = rospy.Publisher('classifier', String, queue_size=10)

  def image_data_callback(self, image_msg, apriltag_msg):
    """
        Using an approximate time synchronizer, take two subcriber topics:
            1. usb camera data (image_msg)
            2. apriltag detection data (apriltag_msg)
        Then perform a machine learning classification on the usb camera data and, depending
        on the results, edit the apriltag detection data and re-publish the edited data
        """

    # convert image data from image message -> opencv image -> numpy array
    cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    np_image = np.asarray(cv_image)
    np_image = np.expand_dims(np_image, axis=0)

    # classify image data using our pre-trained machine learning model
    myclass = self.model.predict(np_image)
    classification = myclass.argmax()

    # publish classification results to topics as neccessary
    msg_class = "Unknown: " + str(classification)
    if classification == 0:
      msg_class = "No cube"
    elif classification == 1:
      msg_class = "good"
    elif classification == 2:
      msg_class = "bad"

    # always print the classification of the image from the usb camera as we get it, for debugging purposes
    self.classifier_publisher.publish('classification = ' + msg_class)
    self.int_classifier_publisher.publish(classification)


if __name__ == '__main__':
  """ initialize and start ROS node object """
  try:
    robot_name = rospy.get_namespace().strip('/')
    rospy.init_node(robot_name + '_TENSORFLOW_CLASSIFIER')
    Classifier(robot_name)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
