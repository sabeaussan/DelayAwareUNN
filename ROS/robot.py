import numpy as np


# TODO : extend serial Robot

# rajouter mask

braccio = {
  "range" : 43.5,
  "lengths" : np.array([7.39,12.61,12.39,10.87]),
  "offsets" : np.array([-30,-2,6,-5,0.0,0.0]),
  "init_joints_state" : np.array([90.0,90.0,90.0,90.0,90.0,30.0])
}



DoF_2 = {
  "range" : 38,
  "lengths" : np.array([8.0,12.0,12.4,5.6]),
  "offsets" : np.array([10,3,3,-27,0,0]),
  "init_joints_state" : np.array([90.0,90.0,90.0,90.0,90.0,25.0])
}


robot_param = [braccio,DoF_2]


class SerialRobot():
  def __init__(self,robot_id,is_2DoF,mask =[0,1,1,1,0,0]) :
    # TODO : mask permet de savoir quelles angles sont utilises
    self.robot_id = robot_id
    self.is_2DoF = is_2DoF
    self.proportions = robot_param[robot_id]["lengths"]/robot_param[robot_id]["range"]#self.init_proportions(lengths,arm_range)
    #self.joints_offsets = np.array([0,0,-8,10,0,0]) * (np.pi/180.0)
    self.mask = mask
    self.offsets = robot_param[robot_id]["offsets"]
    self.init_joints_state = robot_param[robot_id]["init_joints_state"]

  def forward_kinematic(self,joint_states): 
    #if joint_states[0] is None : # 2D problem
    parent_angle = 0
    joint_states = joint_states[np.nonzero(self.mask)]
    x = 0
    y = 0
    for i in range(len(joint_states)):
      x += self.proportions[i+1] * np.sin(joint_states[i] + parent_angle)
      y += self.proportions[i+1] * np.cos(joint_states[i] + parent_angle)
      parent_angle += joint_states[i]
    y += self.proportions[0]
    return np.array([x,y])

  def inverse_kinematic(self,target_pos):
    # TODO : ajouter un check pour voir si on est proche de la target ou pas
    consigne = np.array([0.0,0.0,0.0])
    x3 = target_pos[0]
    y3 = target_pos[1] - self.proportions[0] - self.proportions[3]
    r3_sqr = x3**2 + y3**2
    psi3 = np.arccos((self.proportions[1]**2 + self.proportions[2]**2 - r3_sqr)/(2*self.proportions[1]*self.proportions[2]))
    consigne[1] = np.pi - psi3
    psi2 = np.arctan(x3/y3)
    consigne[0] = psi2 - np.arctan((self.proportions[2]*np.sin(consigne[1]))/(self.proportions[1]+self.proportions[2]*np.cos(consigne[1])))
    consigne[2] =  -consigne[0] - consigne[1]
    consigne = np.clip((consigne*(180/np.pi) + 90),0,180)
    if np.isnan(consigne)[0]:
      print("NAN !!!")
      consigne[0] = 131
      consigne[1] = 152
      consigne[2] = 0
    self.init_joints_state[np.nonzero(self.mask)] = 180 - consigne if self.robot_id == 1 else consigne
    self.init_joints_state[2] = consigne[1]
    joints_state = self.init_joints_state + self.offsets
    joints_state = np.clip(joints_state,0,180)
    return joints_state

  def inverse_kinematic_2DoF(self,target_pos):
    # max = 0.99 min = 0.7
    # -self.proportions[1] pour le braccio 2D
    consigne = np.array([0.0,0.0])
    tmp = (target_pos[1]-self.proportions[0]-self.proportions[2]-self.proportions[3])/self.proportions[1]
    consigne[0] = np.arccos(tmp)
    print("consigne 0 ",consigne[0])
    consigne[1] = - consigne[0]
    consigne = np.clip((consigne*(180/np.pi) + 90),0,180)
    consigne[1] = 180 - consigne[1]
    if np.isnan(consigne)[0]:
      print("NAN !!!")
      consigne[0] = 180
      consigne[1] = 0
    self.init_joints_state[np.nonzero(self.mask)] = consigne
    joints_state = self.init_joints_state + self.offsets
    joints_state = np.clip(joints_state,0,180)
    print(joints_state)
    return joints_state



