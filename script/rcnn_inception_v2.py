#!/usr/bin/env python2
import os
import sys
import time
import random
import numpy as np
import cv2
import roslib
import rospy
from ros_img.srv import return_data, return_dataResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_process import ImageProcess

class RcnnInceptionV2(ImageProcess):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000,
                 rcnn_datapath="/home/lucas/catkin_ws/src/ros_img/script/mask-rcnn-coco",
                 weights="frozen_inference_graph.pb",
                 config="mask_rcnn_inception_v2_coco_2018_01_28.pbtxt",
                 labels="object_detection_classes_coco.txt",
                 colors="colors.txt"):

        if name is None:
            name='rcnn_inception_v2'

        super(RcnnInceptionV2, self).__init__(name=name,
                                              rate=rate,
                                              delta_t_buffer_size=delta_t_buffer_size)

        # derive the paths to the Mask R-CNN weights and model configuration
        self.rcnn_datapath = rcnn_datapath
        self.weights = weights
        self.config = config
        self.labels = labels
        self.colors = colors

        self.weights_datapath = os.path.sep.join([rcnn_datapath, weights])
        self.config_datapath = os.path.sep.join([rcnn_datapath, config])
        self.labels_datapath = os.path.sep.join([rcnn_datapath, labels])
        self.colors_datapath = os.path.sep.join([rcnn_datapath, colors])

        print(cv2.__version__)
        self.nn = cv2.dnn.readNetFromTensorflow(self.weights_datapath , self.config_datapath)
        self.labels = open(self.labels_datapath).read().strip().split("\n")
        COLORS = open(self.colors_datapath).read().strip().split("\n")
        COLORS = [np.array(c.split(",")).astype("int") for c in COLORS]
        self.colors = np.array(COLORS, dtype="uint8")

    # ----------------------------------------------------------------------------------------
    # Main Loop 
    def main_process(self):
        # -----------------------------------
        frame = self.input_frame.copy()
        # **************************
        (H, W) = frame.shape[:2]
        
        blob = cv2.dnn.blobFromImage(frame, swapRB=True, crop=False)
        self.nn.setInput(blob)

        start = rospy.get_rostime().nsecs
        (boxes, masks) = self.nn.forward(["detection_out_final", "detection_masks"])
        
        end = rospy.get_rostime().nsecs
        dt_rcnn = end - start
        if dt_rcnn <= 0:
            dt_rcnn = end - start + 2**32

        print("[INFO] Mask R-CNN took {:.6f} seconds".format(dt_rcnn/1e9))

        #-------------------------------------------------------------------------------
        confidenceThreshold = 0.5
        binarizationThreshold = 0.3
        visualize = 0

        frameCopy = frame.copy()

        # loop over the number of detected objects
        for i in range(0, boxes.shape[2]):
            # extract the class ID of the detection along with the confidence
            # (i.e., probability) associated with the prediction
            classID = int(boxes[0, 0, i, 1])
            confidence = boxes[0, 0, i, 2]

            #---------------------------------------------------------------------------
            # filter out weak predictions by ensuring the detected probability
            # is greater than the minimum probability
            if confidence > confidenceThreshold:
                # clone our original image so we can draw on it


                # scale the bounding box coordinates back relative to the
                # size of the image and then compute the width and the height
                # of the bounding box
                box = boxes[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = box.astype("int")
                boxW = endX - startX
                boxH = endY - startY

                # extract the pixel-wise segmentation for the object, resize
                # the mask such that it's the same dimensions of the bounding
                # box, and then finally threshold to create a *binary* mask
                mask = masks[i, classID]
                mask = cv2.resize(mask, (boxW, boxH), interpolation=cv2.INTER_NEAREST)
                mask = (mask > binarizationThreshold)

                # extract the ROI of the image
                frameRoi = frameCopy[startY:endY, startX:endX]

                #-----------------------------------------------------------------------
                # check to see if are going to visualize how to extract the
                # masked region itself
                if visualize > 0:
                    # convert the mask from a boolean to an integer mask with
                    # to values: 0 or 255, then apply the mask
                    visMask = (mask * 255).astype("uint8")
                    instance = cv2.bitwise_and(frameRoi, frameRoi, mask=visMask)

                    # show the extracted ROI, the mask, along with the
                    # segmented instance 
                    frameRoi = cv2.cvtColor(frameRoi, cv2.COLOR_BGR2RGB)
                    # visMask = cv2.cvtColor(visMask, cv2.COLOR_BGR2RGB)
                    instance = cv2.cvtColor(instance, cv2.COLOR_BGR2RGB)
                    
                    print("ROI:")
                    plt.imshow(frameRoi)
                    plt.show()
                    
                    print("Mask:")
                    plt.imshow(visMask)
                    plt.show()
                    
                    print("Segmented:")
                    plt.imshow(instance)
                    plt.show()
                    

                #-----------------------------------------------------------------------
                # now, extract *only* the masked region of the ROI by passing
                # in the boolean mask array as our slice condition
                frameRoi = frameRoi[mask]

                # randomly select a color that will be used to visualize this
                # particular instance segmentation then create a transparent
                # overlay by blending the randomly selected color with the ROI
                color = random.choice(self.colors)
                blended = ((0.4 * color) + (0.6 * frameRoi)).astype("uint8")

                # store the blended ROI in the original image
                frameCopy[startY:endY, startX:endX][mask] = blended

                # draw the bounding box of the instance on the image
                color = [int(c) for c in color]
                cv2.rectangle(frameCopy, (startX, startY), (endX, endY), color, 2)

                # draw the predicted label and associated probability of the
                # instance segmentation on the image
                text = "{}: {:.4f}".format(self.labels[classID], confidence)
                cv2.putText(frameCopy, text, (startX, startY - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # -----------------------------------
        # frameCopy = cv2.cvtColor(frameCopy, cv2.COLOR_BGR2RGB)
        # **************************
        self.output_frame = frameCopy.copy()

# ======================================================================================================================
def rcnn_inception_v2():
    rcnn_inception_v2 = RcnnInceptionV2()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            rcnn_inception_v2.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            rcnn_inception_v2.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            rcnn_inception_v2.output_frame_publisher_init(rostopic_name=sys.argv[2])
            rcnn_inception_v2.input_frame_subscriber_init()
        else:
            rcnn_inception_v2.input_frame_subscriber_init()
            rcnn_inception_v2.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            rcnn_inception_v2.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            rcnn_inception_v2.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            rcnn_inception_v2.output_frame_publisher_init(rostopic_name=sys.argv[2])
            rcnn_inception_v2.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            rcnn_inception_v2.input_frame_subscriber_init()
            rcnn_inception_v2.output_frame_publisher_init()

    else:
        rcnn_inception_v2.input_frame_subscriber_init()
        rcnn_inception_v2.output_frame_publisher_init()
    
    rcnn_inception_v2.delta_t_service_init()
    rcnn_inception_v2.main_loop()

if __name__ == '__main__':
    try:
        rcnn_inception_v2()
    except rospy.ROSInterruptException:
        pass
