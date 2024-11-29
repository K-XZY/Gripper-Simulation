import pybullet as p
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils.thing import Thing
import os
import csv

# ----------------------------------------------- gripper class -----------------------------------------------

# The gripper class consists all the functions that handle gripper manipulations such as open, grasp, lift mechanism
class gripper:
  def __init__(self, x, y, z, x_gripper, y_gripper, z_gripper, x_ori, y_ori, z_ori, force, lifting_distance, filePath) -> None:
    self.x = x
    self.y = y
    self.z = z     
    self.filePath = filePath
    self.x_gripper = x_gripper
    self.y_gripper = y_gripper
    self.z_gripper = z_gripper
    self.x_ori = x_ori
    self.y_ori = y_ori
    self.z_ori = z_ori
    self.force = force
    self.lifting_distance = lifting_distance

  # perform grasping mechanism
  def grasp(self):
    done = False
    while not done:
      for i in [1,4]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=1)
      p.setJointMotorControl2(self.robot_model, 7, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=2)
      done = True
    self.open = False

  # perform preshape mechanism as a preparation for grasp
  def preshape(self):
    done = False
    while not done:
      for i in [2,5,8]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.4, maxVelocity=2,force=1)
      done = True
    self.open = False

  # Open the gripper during initialisation
  def openGripper(self):
    closed = True
    iteration = 0
    while(closed and not self.open):
      joints = self.getJointPosition()
      closed = False
      for k in range(0,self.numJoints):
        #lower finger joints
        if k==2 or k==5 or k==8:
          goal = 0.9
          if joints[k] >= goal:    
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05, 
                                    maxVelocity=2,force=5)   
            closed = True
        #Upper finger joints             
        elif k==6 or k==3 or k==9:
          goal = 0.9
          if joints[k] <= goal:
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05,
                                    maxVelocity=2,force=5)
            closed = True
        #Base finger joints
        elif k==1 or k==4 or k == 7:
          pos = 0.9
          if joints[k] <= pos:
            p.setJointMotorControl2(self.robot_model, k, p.POSITION_CONTROL,
                                    targetPosition=joints[k] - 0.05,
                                    maxVelocity=2,force=5)
            closed = True
      iteration += 1
      if iteration > 1000:
        break
      p.stepSimulation()
    self.open = True

  # Read the joint positions
  def getJointPosition(self):
    joints = []
    for i in range(0, self.numJoints):
      joints.append(p.getJointState(self.robot_model, i)[0])
    return joints
        
  # Initialisation for each experiment
  def experiment_setup(self):
    self.initial_position = [self.x+self.x_gripper, self.y+self.y_gripper, self.z+self.z_gripper]
    self.initial_orientation = p.getQuaternionFromEuler([self.x_ori, self.y_ori, self.z_ori])
    self.gripperId = p.loadURDF(self.gripperName, self.initial_position, self.initial_orientation, 
                                globalScaling=1, useFixedBase=False)
    p.setRealTimeSimulation(1)
    self.hand_base_controller = p.createConstraint(self.gripperId,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      p.JOINT_FIXED,
                                                      [0, 0, 0], [0, 0, 0], self.initial_orientation)
    p.changeConstraint(self.hand_base_controller, self.initial_position, 
                         jointChildFrameOrientation=self.initial_orientation, maxForce=self.force)
    self.open = False
    self.numJoints = p.getNumJoints(self.gripperId)
    self.robot_model = self.gripperId
  
  # draw x y z axis at the origin
  def draw_base_axis(self):
    # let's visualize the world reference frame
    p.addUserDebugLine([0, 0, 0], [0.5, 0, 0], [1, 0, 0], 1, 0) # lookup the parameters of this!
    p.addUserDebugLine([0, 0, 0], [0, 0.5, 0], [0, 1, 0], 1, 0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.5], [0, 0, 1], 1, 0)
    
  # spawn an object
  def object_spawn(self, objectName):  
    self.cube_id = p.loadURDF(objectName, [self.x, self.y, self.z], self.initial_orientation, globalScaling=2)
    cube_position_before = p.getBasePositionAndOrientation(self.cube_id)[0]
    
  # lift the gripper by the lifting distance
  def lifting(self):
    p.changeConstraint(self.hand_base_controller, [self.initial_position[0], self.initial_position[1], self.initial_position[2] + self.lifting_distance] , 
                         jointChildFrameOrientation=self.initial_orientation, maxForce=self.force)
    
# ----------------------------------------------- experiment class -----------------------------------------------

# a class that inherits from the gripper class with extra function to perform grasp success validation
class experiment(gripper):
  def __init__(self, x, y, z, gripperName, object, filePath) -> None: 
    self.gripperName = gripperName
    self.object = object
    self.outcome = None # 1 being successful and 0 being fail
    # Generate random offsets and orientation of the gripper from the origin coordinate of the experiment x, y, z
    x_gripper = np.random.uniform(-0.075, 0.075)
    y_gripper = np.random.uniform(-0.075, 0.075)
    z_gripper = np.random.uniform(0.15, 0.2)
    x_ori = np.random.normal(loc=np.pi, scale=0.2)
    y_ori = np.random.normal(loc=0, scale=0.2)
    z_ori = np.random.uniform(0, 2 * math.pi)
    force = 75
    lifting_distance = 1.5
    super().__init__(x, y, z, x_gripper, y_gripper, z_gripper, x_ori, y_ori, z_ori, force, lifting_distance, filePath)
    
  # a function to check if the object is successfully grapsed and lifted by the gripper
  def check_valid_grasp(self):
    cube_position_after2 = p.getBasePositionAndOrientation(self.cube_id)[0]
    gripper_pos = p.getBasePositionAndOrientation(self.gripperId)[0]
    print(cube_position_after2[2])
    print(abs(gripper_pos[2]-cube_position_after2[2]))
    
    if cube_position_after2[2]>self.lifting_distance-0.5 and abs(gripper_pos[2]-cube_position_after2[2]) < 0.25:
      print("\033[32mThe cube was successfully lifted!\033[0m")
      with open(self.filePath, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([self.x_gripper,self.y_gripper, self.z_gripper, self.x_ori, self.y_ori, self.z_ori, 1])
      self.outcome = 1
      return 1
    else:
      print("\033[31mThe cube was not lifted. Grasp might have failed.\033[0m")
      with open(self.filePath, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([self.x_gripper,self.y_gripper, self.z_gripper, self.x_ori, self.y_ori, self.z_ori, 0])
      self.outcome = 0
      return 0
    
# ----------------------------------------------- BatchExperiment class -----------------------------------------------

# a class that instantiate n (numberOfExperimentPerBatch) experiment objects to perform grasping experiment simultaneously
class BatchExperiment():
  def __init__(self, batchNumber, numberOfExperimentPerBatch, gripperName, graspObject, folderName, fileName):
    self.batchNumber = batchNumber
    self.numberOfExperimentPerBatch = numberOfExperimentPerBatch
    self.gripperName = gripperName
    self.graspObject = graspObject
    self.folderName = folderName
    self.fileName = fileName
    self.filePath = os.path.join(folderName, fileName)
  
  # create a folder and a file to store the experiment data
  def initialiseCSV(self):
    # Check if the folder exists, create if it doesn't
    if not os.path.exists(self.folderName):
        os.makedirs(self.folderName)
        print(f"Folder '{self.folderName}' created.")
    else:
        print(f"Folder '{self.folderName}' already exists.")

    # Open a CSV file for writing
    if self.batchNumber == 0:
      with open(self.filePath, mode="w", newline="", encoding="utf-8") as file:
          writer = csv.writer(file)

          # Write the header
          writer.writerow(["x", "y", "z", "x_ori", "y_ori", "z_ori", "result"])  
    else: 
      with open(self.filePath, mode="a", newline="", encoding="utf-8") as file:
          writer = csv.writer(file)


  # connect to pybullet user interface
  def connectToPyBullet(self):
    clid = p.connect(p.SHARED_MEMORY)
    if (clid < 0):
        p.connect(p.GUI)
    return
  
  # function to generate origin corrdinate of n test space frame
  def generate_grid_coordinates(self):
    # Ensure the grid fits all points
    size = int(np.ceil(np.sqrt(self.numberOfExperimentPerBatch)))  
    x, y = np.meshgrid(range(size), range(size))
    coordinates = np.array(list(zip(x.flatten(), y.flatten())))
    # Return only the first n points 
    return coordinates[:self.numberOfExperimentPerBatch]  
  
  # perform multiple experiment object initialisations and store them in a list
  def experimentSpaceInitialisation(self, experiment_space_coordinates, experiment_spaces):
    for x, y in experiment_space_coordinates:
        new_experiment_space = experiment(x, y, 0, self.gripperName, self.graspObject, self.filePath)
        new_experiment_space.experiment_setup()
        experiment_spaces.append(new_experiment_space) 
    return

  # perform simultaneous grasping experiments
  def batchGraspingOperation(self, experiment_spaces):
    #open and close the fingers  
    time.sleep(1)
    for experiment_space in experiment_spaces:
      experiment_space.openGripper()
    time.sleep(0.5)
    for experiment_space in experiment_spaces:
      experiment_space.preshape()
    for experiment_space in experiment_spaces:
      experiment_space.object_spawn(self.graspObject)
    # grasp
    time.sleep(0.1)
    for experiment_space in experiment_spaces:
      experiment_space.grasp()
    #lift the object
    time.sleep(2)
    for experiment_space in experiment_spaces:
      experiment_space.lifting()
    time.sleep(1)
    # Get cube position for the second time to check if the block is moving
    time.sleep(1.5)
    return
    
  # a function that triggers the overal operation for a batch
  def batchOperation(self): 
    self.initialiseCSV()
    # Connect to Pybullet
    self.connectToPyBullet() 
    # Create experiment spaces (including the grippers and grasping objects)
    experiment_spaces = []
    experiment_space_corrdinates = self.generate_grid_coordinates()      
    self.experimentSpaceInitialisation(experiment_space_corrdinates, experiment_spaces)  
    # Perform grasping operation in a batch
    self.batchGraspingOperation(experiment_spaces)  
    # Check each result and write to data file
    for experiment_space in experiment_spaces:
      experiment_space.check_valid_grasp()
    # disconnect to Pybullet
    p.disconnect()
    return
    
# ----------------------------------------------- main -----------------------------------------------
def main():
  # hyperParameters:
  # currently only support Three Finger for gripper and cube_small for graspObject, different grippers and objects can be used by changing their path.
  gripperName = "./Robots/grippers/threeFingers/sdh.urdf"
  graspObject = "./share/cube_small.urdf"
  numberOfBatch = 500
  numberOfExperimentPerBatch = 10 # recommended maximum number is 10 for stable simulation
  folderName = 'OOP_data_folder'
  fileName = 'OOP_data.csv'


  for i in range(numberOfBatch):
    currentBatch = BatchExperiment(i, numberOfExperimentPerBatch, gripperName, graspObject, folderName, fileName)
    currentBatch.batchOperation()
    

if __name__ == '__main__':
  main()