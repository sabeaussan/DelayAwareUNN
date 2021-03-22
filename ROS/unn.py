#!/usr/bin/env python
import rospy
import numpy as np
import argparse
import Queue
from robot import SerialRobot
from agent import Agent
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray,Float64MultiArray
from gutter_task.msg import Obs
import time

# Equilibrium normalized effector position
POSE_EFFECTOR_ORIGIN_0 = [0.46,0.62]
POSE_EFFECTOR_ORIGIN_1 = [0.0,0.91] 
POSE_EFFECTOR_ORIGIN = [POSE_EFFECTOR_ORIGIN_0,POSE_EFFECTOR_ORIGIN_1]

# Scale factor
UNN_SCALE_0 = 0.15
UNN_SCALE_1 = 0.1
UNN_SCALE = [UNN_SCALE_0,UNN_SCALE_1]

# Effector min and max height
EFFECTOR_RANGE_0 = [0.51,0.73]
EFFECTOR_RANGE_1 = [0.82,0.99]
EFFECTOR_RANGE = [EFFECTOR_RANGE_0,EFFECTOR_RANGE_1]

DELAY = 0.3 # Delay considered for the system in second and fed to the Delay Aware model
ADDED_DELAY = 4 # 1 sample = 100 ms, delay added on the system
CHANGE_FREQ = 150 # Change desired ball position every 150 steps
old_ball_position = -1

# Desired ball positions for performance test (long)
DESIRED_BALL_POSITION = [0.665146, 0.319444, 0.210018, 0.788669, 0.625432, 0.413984,0.492742, 0.217814, 0.592103, 0.709749, 0.573616, 0.787164, 0.263239, 0.270857, 0.546654, 0.211952, 0.586792, 0.629471,0.333639, 0.20229, 0.614752, 0.785692, 0.240727, 0.304158, 0.515807, 0.379363, 0.716785, 0.487727, 0.360066, 0.506695, 0.680391,0.671943, 0.551617, 0.494168, 0.562634, 0.271272, 0.648575, 0.322888,0.368295, 0.645345, 0.274886, 0.637379, 0.480301, 0.286186,0.483854, 0.252525, 0.653358, 0.720112, 0.433729, 0.775511]

# Desired ball positions for short test
TEST_VEC = [0.3,0.8,0.5]

TEST = [TEST_VEC,DESIRED_BALL_POSITION]

t1 = 0
t2 = 0

def subscriber_callback(obs):
  global observations
  global received_obs
  observations = obs 
  if obs is not None : 
    received_obs = True


def print_step(obs,unn_inst,actions,des_pos,step):
  # Display all informations about this timestep
  print("################## NB STEP : {} ####################".format(step))
  print("Desired ball position : {}".format(obs[0]))
  print(" ball position : {}".format(obs[1]))
  print("ball velocity : {}".format(obs[2]))
  print(" unn instruction : {}".format(unn_inst))
  print("effec_pos : {}".format(obs[3]))
  print("Desired effec pos : {}".format(des_pos))
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

def save_log(obs,suffix,dir_path):
  np.savetxt(dir_path+"/log_observations_"+suffix+".txt",np.array(obs))

def computeError(obs):
  print(obs.shape)
  return np.trapz(np.abs(obs[:,0]-obs[:,1]))

def unn(args):
  global delayed_obs

  print(UNN_SCALE[args.robot])
  print(POSE_EFFECTOR_ORIGIN[args.robot])
  print(EFFECTOR_RANGE[args.robot])
  # Path for loading the UNN tf model
  PATH_UNN = "/models/.pb" # renommer en delay aware 
  if not args.method == 0  :
    PATH_UNN = "/home/samuel/catkin_ws/src/ros/utils/saved_models/GutterBAM_no_delay.pb"

  # Instantiate the agent with the tf model
  unn = Agent(PATH_UNN)

  # ROS nodes init
  sub = rospy.Subscriber("observations",Obs,subscriber_callback,queue_size = 1)

  # 
  if args.robot == 1 :
    pub = rospy.Publisher("/usb2servo/joint_angles", Float64MultiArray, queue_size=1)
  else :
    pub = rospy.Publisher("joint_angles", Float32MultiArray, queue_size=1)
  
  rospy.init_node('UNN')
  rate = rospy.Rate(10)
  array = Float64MultiArray() if args.robot == 1 else Float32MultiArray()
  fifo_obs = Queue.Queue()

  # init log buffer
  buffer_observations = []
  buffer_delayed_observations = []


  nb_step = 0
  # total test length is number of desired position * duration 
  test_length = CHANGE_FREQ * len(TEST_VEC) if args.test == 0 else CHANGE_FREQ * len(DESIRED_BALL_POSITION)


  is_2DoF = args.robot == 1
  robot = SerialRobot(args.robot,is_2DoF)
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
        # feed the UNN model with the observations to get the desired effector position
        unn_instruction = unn.act(delayed_obs)
        # clip the command 
        unn_instruction_clipped = np.clip(unn_instruction,-1,1)

        # scale the desired effector position
        if unn_instruction_clipped <= 0 :
          unn_instruction_clipped = unn_instruction_clipped * UNN_SCALE[args.robot]
        else :
          unn_instruction_clipped = unn_instruction_clipped * UNN_SCALE[args.robot]
        unn_instruction_clipped = np.round(unn_instruction_clipped,2)

        # compute the desired normalized effector position
        desired_effec_pos = [POSE_EFFECTOR_ORIGIN[args.robot][0],np.clip(unn_instruction_clipped[0]+POSE_EFFECTOR_ORIGIN[args.robot][1],EFFECTOR_RANGE[args.robot][0],EFFECTOR_RANGE[args.robot][1])]
        desired_effec_pos = np.round(desired_effec_pos,2)

        # compute corresponding joints angle with inverse kinematic
        if robot.is_2DoF : 
          actions = robot.inverse_kinematic_2DoF(desired_effec_pos) # a clip
          actions[1] = actions[2]
        else : 
          actions = robot.inverse_kinematic(desired_effec_pos)

        # published the actions
        publish_action(pub,array,actions)


        #nb_step+=1
        #tmp = list(obs)
        #if args.method == 0:
        #  del tmp[1]
        #print_step(tmp,unn_instruction_clipped,actions,desired_effec_pos,nb_step)

        if len(buffer_observations) == test_length - ADDED_DELAY and args.log is not None:
          save_log(buffer_delayed_observations,args.log,"logs")
          score = computeError(np.array(buffer_observations))
          print("##### Perf pour le delay {} : {} #####".format(DELAY,score/len(DESIRED_BALL_POSITION)))
          break
      else : 
        print("delayed")
    rate.sleep()



if __name__=="__main__" :
  parser = argparse.ArgumentParser()
  parser.add_argument("-p","--model_path",help= "path to the model" ,type = str)
  parser.add_argument("-m","--method",help= "Should use delay aware model 0 (yes), 1 (no)" ,type = int,default = 0)
  parser.add_argument("-r","--robot",help= "Robot chosen : 0 (Braccio), 1 (2 DoF)" ,type = int,default = 0)
  parser.add_argument("-l","--log",help= "Should log the run, you need to specify run name" ,type = str,default = None)
  parser.add_argument("-t","--test",help= "What kind of test should be runned : short test (0) or long test (1)" ,type = int,default = 0)
  args = parser.parse_args()
  try:
    unn(args)
  except rospy.ROSInterruptException:
    pass
