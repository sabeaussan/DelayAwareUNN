#!/usr/bin/env python
import rospy
import random
import numpy as np
import cv2
import time
import math
from gutter_task.msg import Obs

# Normalizing constant
Y_EFF_REF = 217# 0.62
Y_EFF_MAX = 255 #0.74
Y_EFF_MIN = 180 # 0.50
GUTTER_LENGTH = 330#np.loadtxt("gutter_length.txt",delimiter=",")  # pixel plane donc a recalibrer a chaque fois


old_ball_position = -1
CAMERA_HEIGHT = 360
CAMERA_WIDTH = 640

print("GUTTER_LENGTH : {}".format(GUTTER_LENGTH))

# Threshold mask range
LOWER_BOUND_YELLOW = (15,68,122)
UPPER_BOUND_YELLOW = (38,131,197)

LOWER_BOUND_GREEN = (64,92,28)
UPPER_BOUND_GREEN = (94,203,151)


def track_ball_and_effector(frame,desired_ball_position):
  # track both the ball and the effector position
  # blur the frame (smooth noise) and convert it to the HSV color space
  blurred = cv2.GaussianBlur(frame, (11, 11), 0)
  hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
  # construct a mask for the colors, then perform
  # a series of dilations and erosions to remove any small
  # blobs left in the mask
  mask_effector = cv2.inRange(hsv, LOWER_BOUND_YELLOW, UPPER_BOUND_YELLOW)
  mask_effector = cv2.erode(mask_effector, None, iterations=2)
  mask_effector = cv2.dilate(mask_effector, None, iterations=2)

  mask_ball = cv2.inRange(hsv, LOWER_BOUND_GREEN, UPPER_BOUND_GREEN)
  mask_ball = cv2.erode(mask_ball, None, iterations=2)
  mask_ball = cv2.dilate(mask_ball, None, iterations=2)

  _,contours_effector,_ = cv2.findContours(mask_effector.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  _,contours_ball,_ = cv2.findContours(mask_ball.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

  # only proceed if at least one contour was found for both
  if len(contours_effector) > 0 and len(contours_ball)>0: 
  # find the largest contour in the masks, then use
  # it to compute the minimum enclosing circle 
    c_effector = max(contours_effector, key=cv2.contourArea)
    c_ball = max(contours_ball, key=cv2.contourArea)
    ((x_eff, y_eff), radius_eff) = cv2.minEnclosingCircle(c_effector)
    ((x_ball,y_ball), radius_ball) = cv2.minEnclosingCircle(c_ball)
    cv2.circle(frame,(int(x_eff + desired_ball_position * GUTTER_LENGTH), int(CAMERA_HEIGHT- Y_EFF_REF - 10)), int(5),(255, 50, 50),12) # blue ball desired pos
    cv2.circle(frame, (int(x_eff), int(y_eff)), int(radius_eff),(0, 255, 255), 2)    # yellow circle effector
    cv2.circle(frame, (int(x_ball), int(y_ball)), int(radius_ball),(0, 255, 255), 2) # yellow circle ball
    if radius_eff > 5 and radius_ball> 5:
      # only proceed if the radius meets a minimum size
      #cv2.ellipse(frame,(int(x_eff), int(y_eff)), (int(radius_des_ball),int(radius_des_ball)),-2,startAngle,endAngle,(0,0,255), 10)
      return [x_eff,y_eff],[x_ball,y_ball]
    
def compute_ball_local_position(ball_position,origin):
  # find the ball local position in the gutter 
  # by computing the distance between the effector and the ball then normalize 
  x = (ball_position[0] - origin[0]) **2
  y = (ball_position[1] - origin[1]) **2
  local_position = math.sqrt(x+y)
  return (local_position / GUTTER_LENGTH )

  

def main():

  # OpenCV init
  # load camera's calibration parameters
  camera_matrix = np.loadtxt("cam_mtx.txt",delimiter=",")
  camera_distorsion = np.loadtxt("disorsion_param.txt",delimiter=",")
  # get webcam image
  cam = cv2.VideoCapture(0)
  cam.set(cv2.CAP_PROP_FRAME_WIDTH,640)
  cam.set(cv2.CAP_PROP_FRAME_HEIGHT,380)
  # allow the camera to warm up
  time.sleep(3.0)
  ball_position = 0
  effector_position = 0
  
  # ROS init
  pub = rospy.Publisher('observations', Obs, queue_size=1)
  rospy.init_node('camera')
  rate = rospy.Rate(30)
  msg = Obs()
  cnt = 0
  idx = 0
  position_des = [0.4,0.8,0.5]
  while not rospy.is_shutdown():

    # Read the camera frame
    ret, frame = cam.read()

    # Get useful observations
    observations  = track_ball_and_effector(frame,position_des[idx])

    if observations is not None:

      # counter for displaying the blue ball
      if cnt%450 ==0 and idx < 2:
        idx = cnt/450

      # get ball position
      ball_local_position = compute_ball_local_position(observations[1],observations[0])

      # build the message and publish
      msg.effector_position_y = (float) ((-observations[0][1] + CAMERA_HEIGHT - Y_EFF_REF)/(Y_EFF_MAX - Y_EFF_REF))
      msg.ball_position = (float) (ball_local_position)
      msg.ball_velocity = (float) (0)
      msg.ball_desired_position = (float) (position_des[idx])
      pub.publish(msg)
      print(msg)
      
      cnt+=1
      
    # Display the frame 
    cv2.imshow('frame',frame)
    rate.sleep()

    # use q to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
      cam.release()
      cv2.destroyAllWindows()
      break

    

if __name__=="__main__" :
  try:
    main()
  except rospy.ROSInterruptException:
    pass
