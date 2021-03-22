#!/usr/bin/env python
import rospy
import numpy as np
import Queue
import argparse
from agent import Agent
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray,Float32MultiArray
from gutter_task.msg import Obs
from utils import extract_episode_reward,open_log_file,extract_position,extract_episode_obs
import time


t1 = 0
t2 = 0

old_ball_position = -1

DELAY = 0.3
ADDED_DELAY = 0# 1 sample = 100 ms
CHANGE_FREQ = 80

REF_POSITION_JOINTS_0 = [125,149,0]
REF_POSITION_JOINTS_1 = [137,107]
REF_POSITION_JOINTS = [REF_POSITION_JOINTS_0,REF_POSITION_JOINTS_1]


PATH_MODEL_0 = ["/home/samuel/catkin_ws/src/ros/utils/saved_models/BraccioVanillaDA.pb","/home/samuel/catkin_ws/src/ros/utils/saved_models/BraccioVanillaDU.pb"]
PATH_MODEL_1 = ["/home/samuel/catkin_ws/src/ros/utils/saved_models/2DoFVanillaDA.pb","/home/samuel/catkin_ws/src/ros/utils/saved_models/2DoFVanillaDU.pb"]
PATH_MODEL = [PATH_MODEL_0,PATH_MODEL_1]


DESIRED_BALL_POSITION = [0.665146, 0.319444, 0.210018, 0.788669, 0.625432, 0.413984,0.492742, 0.217814, 0.592103, 0.709749, 0.573616, 0.787164, 0.263239, 0.270857, 0.546654, 0.211952, 0.586792, 0.629471,0.333639, 0.20229, 0.614752, 0.785692, 0.240727, 0.304158, 0.515807, 0.379363, 0.716785, 0.487727, 0.360066, 0.506695, 0.680391,0.671943, 0.551617, 0.494168, 0.562634, 0.271272, 0.648575, 0.322888,0.368295, 0.645345, 0.274886, 0.637379, 0.480301, 0.286186,0.483854, 0.252525, 0.653358, 0.720112, 0.433729, 0.775511]

TEST_VEC = [0.3,0.8,0.5]

def subscriber_callback(obs):
  global observations
  global received_obs
  observations = obs 
  if obs is not None : 
    received_obs = True

def print_step(obs,actions,step):
  print("################## NB STEP : {} ####################".format(step))
  print(" efec pos : {}".format(obs[3]))
  print(" desired ball position : {}".format(obs[0]))
  print("ball pos : {}".format(obs[1]))
  print("ball vel : {}".format(obs[2]))
  print(" actions : {}".format(actions))
  print("#####################################################################")

def get_observations(step,args):
  # The observations from the camera ros node
  # they are concatenated into a list to be fed to the tf model
  # compute the ball velocity as a difference between position from old to new frame
  global old_ball_position
  global t1
  global t2
  t1 = t2
  t2 = time.time()
  delta = t2 - t1 # approximatelty 0.1s
  if observations is not None : 
    ball_pos = observations.ball_position
    ball_vel = (ball_pos - old_ball_position)/(delta*0.9)#np.clip(observations.ball_velocity,-1,1)
    old_ball_position = ball_pos
    effec_pose = np.round(observations.effector_position_y,2)
    ball_pos = np.round(ball_pos,3)
    ball_vel = np.round(ball_vel,3)
    desired_ball_pos = DESIRED_BALL_POSITION[step] if args.test == 1 else observations.ball_desired_position
    ret = [desired_ball_pos,DELAY,ball_pos,ball_vel,effec_pose] if args.method == 0 else [desired_ball_pos,ball_pos,ball_vel,effec_pose]
    return ret	


def publish_action(publisher,array,actions):
  # Publish the action 
  array.data = []
  for i in range(0,len(actions)):
    if i == 0 :
      array.data.append(actions[i])
    else :
      array.data.append(actions[i])
  print("publishing : {}".format(array.data))
  publisher.publish(array)

observations = None
delayed_obs = None
received_obs = False

def save_log(obs,effec_pos,unn_inst,suffix,dir_path):
  np.savetxt(dir_path+"/log_observations_"+suffix+".txt",np.array(obs))
  np.savetxt(dir_path+"/log_unn_instructions_"+suffix+".txt",np.array(unn_inst))
  np.savetxt(dir_path+"/log_effec_pos_"+suffix+".txt",np.array(effec_pos))


def computeError(obs):
  print(obs.shape)
  return np.trapz(np.abs(obs[:,0]-obs[:,2]))

def vanilla(args): 
  # Instantiate the agent with the tf model
  model = Agent(PATH_MODEL[args.robot][args.method])
  print("started")
  # 
  if args.robot == 1 :
    pub = rospy.Publisher("/usb2servo/joint_angles", Float64MultiArray, queue_size=1)
  else :
    pub = rospy.Publisher("joint_angles", Float32MultiArray, queue_size=1)
  # ROS nodes init
  sub = rospy.Subscriber("observations",Obs,subscriber_callback,queue_size = 1)
  rospy.init_node('vanilla')
  rate = rospy.Rate(10)
  array = Float64MultiArray() if args.robot == 1 else Float32MultiArray()
  fifo_obs = Queue.Queue()

  # init log buffer
  buffer_observations = []
  buffer_delayed_observations = []
  buffer_unn_instructions = []
  buffer_effec_pose = []

  nb_step = 0
  test_length = CHANGE_FREQ * len(TEST_VEC) if args.test == 0 else CHANGE_FREQ * len(DESIRED_BALL_POSITION)

  print("started")

  while not rospy.is_shutdown():
    if received_obs and observations is not None:
      # get observations (delayed by ~ 300 ms)
      obs = get_observations(nb_step/CHANGE_FREQ,args)
      # put the newly obtained observation into a buffer
      fifo_obs.put(obs)
      # if the buffer of size ADDED_DELAY +1 is full
      if fifo_obs.qsize()== ADDED_DELAY + 1:
        # we get the delayed obs
        # if ADDED_DELAY = 0 we always get the newly added obs  
        delayed_obs = fifo_obs.get()
        buffer_observations.append(obs)
        buffer_delayed_observations.append(delayed_obs)

      if delayed_obs is not None :
        # feed the model with the observations to get the joints offsets
        actions = model.act(delayed_obs)
        # scale and clip
        if args.robot == 0 : 
          actions = np.clip(actions,-1,1)
          actions[0] = np.clip(actions[0]*5 + REF_POSITION_JOINTS[args.robot][0],0,180)
          actions[1] = np.clip(actions[1]*15 + REF_POSITION_JOINTS[args.robot][1],0,180)
          actions[2] = np.clip(actions[2]*15 + REF_POSITION_JOINTS[args.robot][2],0,180)
          command = np.clip([60,actions[0] - 2,actions[1] +6,actions[2]-5,90,30],0,180)
        else : 
          actions[0] = np.clip(actions[0],-0.5,0.5)*86 + REF_POSITION_JOINTS[args.robot][0]
          actions[1] = np.clip(actions[1],-1,0.2)*-73 + REF_POSITION_JOINTS[args.robot][1]
          command = np.clip([100,actions[0]+3,actions[0]+3,actions[1]+27,90,25],0,180)
        command = np.rint(command)
        #publish_action(pub,array,command)

        nb_step+=1
        tmp = list(obs)
        if args.method == 0:
          del tmp[1]
        print_step(obs,command,nb_step)

        if len(buffer_observations) == test_length - ADDED_DELAY and args.log is not None:
          save_log(buffer_delayed_observations,buffer_effec_pose,buffer_unn_instructions,"2DoFVanillaDA","logs")
          score = computeError(np.array(buffer_observations))
          print("##### Perf pour le delay {} : {} #####".format(DELAY,score/50))
          break
    rate.sleep()



if __name__=="__main__" :
  parser = argparse.ArgumentParser()
  parser.add_argument("-p","--model_path",help= "path to the model" ,type = str)
  parser.add_argument("-m","--method",help= "Should use delay aware model 0 (yes), 1 (no)" ,type = int,default = 0)
  parser.add_argument("-r","--robot",help= "Robot chosen vg : 0 (Braccio), 1 (2 DoF)" ,type = int,default = 0)
  parser.add_argument("-l","--log",help= "Should log the run, you need to specify run name" ,type = str,default = None)
  parser.add_argument("-t","--test",help= "What kind of test should be runned : short test (0) or long test (1)" ,type = int,default = 0)
  args = parser.parse_args()
  try:
    vanilla(args)
  except rospy.ROSInterruptException:
    pass
