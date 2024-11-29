import pybullet as p
import time
import math
import numpy as np
import pybullet_data

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils.thing import Thing

import csv
import random


# Spawning positions (theta, phi, r)
def spawn_p_spherical_coordinate(r_min:float, r_max:float) -> np.array:
    """
    args:
    r_min (float): Minimum radius of the sphere.
    r_max (float): Maximum radius of the sphere.
    Spawns a random point in spherical coordinates with z > 0.
    """
    theta = np.random.uniform(0, 2 * np.pi)
    phi = np.random.uniform(0, np.pi/2)
    r = np.random.uniform(r_min, r_max)
    return np.array([theta, phi, r])

def convert_spherical_to_euclidean(p: np.array) -> np.array:
    """
    Converts a point in spherical coordinates to Euclidean coordinates.
    
    Parameters:
    p: np.array, shape(3,) - spherical coordinates (theta, phi, r)
    
    Returns:
    np.array, shape(3,) - Euclidean coordinates (x, y, z)
    """
    theta, phi, r = p
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    z = r * np.cos(phi)
    return np.array([x, y, z])

def generate_point_cloud(N=100, op=0.5):
    """
    Generates and plots a 3D point cloud.
    
    Parameters:
    N (int): Number of points to generate.
    op (float): Opacity of the points (0 < op <= 1).
    """
    points = []
    
    for _ in range(N):
        p_spherical = spawn_p_spherical_coordinate(0.15,0.4)
        p_euclidean = convert_spherical_to_euclidean(p_spherical)
        points.append(p_euclidean)
    
    points = np.array(points)
    
    # Plotting the point cloud
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='blue', alpha=op)
    
    # Setting labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Point Cloud')
    
    # Display the plot
    plt.show()

class gripper:

  def grasp(self):
    
    done = False
    while not done:
   
      for i in [1,4]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=1)
        #p.stepSimulation()
      p.setJointMotorControl2(self.robot_model, 7, p.POSITION_CONTROL, 
                                targetPosition=0.05, maxVelocity=1,force=2)
      done = True
    self.open = False

  def preshape(self):
    
    done = False
    while not done:
      
      for i in [2,5,8]:
        p.setJointMotorControl2(self.robot_model, i, p.POSITION_CONTROL, 
                                targetPosition=0.4, maxVelocity=2,force=1)
        #p.stepSimulation()
      done = True
    self.open = False


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
      if iteration > 10000:
        break
      p.stepSimulation()
    self.open = True

  def getJointPosition(self):
    joints = []
    for i in range(0, self.numJoints):
      joints.append(p.getJointState(self.robot_model, i)[0])
    return joints

  def set_init_finger_pose(self):
    self.reset_finger([[0, 0, 0, 0], [0, 0, 0, 0], 
                       [0, 0, 0, 0], [0.7, 0.8, 0.8, 0.8]])

  def reset_finger(self, hand_config):
    for i, finger_index in enumerate(self.fingers):
      for j, joint in enumerate(finger_index):
        p.resetJointState(self.gripperId, joint, hand_config[i][j])

  # takes in the initial cartezion positions and run the grasping simulation and 
  def simiulate_grasp(self, x, y, z, z_ori, force, lifting_distance):
    
    clid = p.connect(p.SHARED_MEMORY)
    if (clid < 0):
      p.connect(p.GUI)
      
    # plane = add_plane()

    path = "../Robots/grippers/threeFingers/sdh.urdf"

    initial_position = [x, y, z]
    initial_orientation = p.getQuaternionFromEuler([np.pi, 0, z_ori])
    
    self.gripperId = p.loadURDF(path, initial_position, initial_orientation, 
                                globalScaling=1, useFixedBase=False)

    p.setRealTimeSimulation(1)
    self.hand_base_controller = p.createConstraint(self.gripperId,
                                                      -1,
                                                      -1,
                                                      -1,
                                                      p.JOINT_FIXED,
                                                      [0, 0, 0], [0, 0, 0], initial_orientation)     

    # let's visualize the world reference frame
    p.addUserDebugLine([0, 0, 0], [0.5, 0, 0], [1, 0, 0], 1, 0) # lookup the parameters of this!
    p.addUserDebugLine([0, 0, 0], [0, 0.5, 0], [0, 1, 0], 1, 0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.5], [0, 0, 1], 1, 0)
    
    # spawn the gripper
    p.changeConstraint(self.hand_base_controller, initial_position, 
                         jointChildFrameOrientation=initial_orientation, maxForce=force)
    
    #open and close the fingers
    self.open = False
    self.numJoints = p.getNumJoints(self.gripperId)
    self.robot_model = self.gripperId
    
    time.sleep(1)
    # input("\033[31menter to grasp and lift the gripper\033[0m")
    self.openGripper()
    time.sleep(0.5)
    self.preshape()
    
    # spawn a cube
    time.sleep(0.5)
    cube_id = p.loadURDF("cube_small.urdf", [0, 0, 0], initial_orientation, globalScaling=2)
    cube_position_before = p.getBasePositionAndOrientation(cube_id)[0]
    
    # grasp
    time.sleep(0.1)
    self.grasp()
    
    #lift the object
    time.sleep(2)
    p.changeConstraint(self.hand_base_controller, [initial_position[0], initial_position[1], initial_position[2] + lifting_distance] , 
                         jointChildFrameOrientation=initial_orientation, maxForce=force)
    time.sleep(1)
    
    # Get cube position after lifting
    # cube_position_after1 = p.getBasePositionAndOrientation(cube_id)[0]

    # Get cube position for the second time to check if the block is moving
    time.sleep(1.5)
    cube_position_after2 = p.getBasePositionAndOrientation(cube_id)[0]
    gripper_pos = p.getBasePositionAndOrientation(self.gripperId)[0]
    
    # Check if the cube's z-coordinate increased by at least `lifting_distance`
    # print(f'cube_pos1: {cube_position_after1}, cube_pos2: {cube_position_after2}')
    print(cube_position_after2[2])
    print(abs(gripper_pos[2]-cube_position_after2[2]))
    # print(cube_position_after2[2]>lifting_distance)
    # print(abs(gripper_pos[2]-cube_position_after2[2]) < 0.2)
    # input("\033[31menter to exit\033[0m")
    p.disconnect()
    
    if cube_position_after2[2]>lifting_distance-0.5 and abs(gripper_pos[2]-cube_position_after2[2]) < 0.25:
        print("\033[32mThe cube was successfully lifted!\033[0m")
        return 1
    else:
        print("\033[31mThe cube was not lifted. Grasp might have failed.\033[0m")
        return 0
    
    


# p_spherical = spawn_p_spherical_coordinate(0.15,2.18)
# theta, phi, r = p_spherical
# print(f'p_spherical: {p_spherical}')
# p_euclidean = convert_spherical_to_euclidean(p_spherical)

# x = np.random.uniform(-0.075, 0.075)
# y = np.random.uniform(-0.075, 0.075)
# z = np.random.uniform(0.15, 0.2)
# z_ori = np.random.uniform(0, 2 * math.pi)
# # force = np.random.uniform(0, 100)
# force = 75
# lifting_distance = 1.5

# # plane = Thing("plane")
# gripper1 = gripper()
# gripper1.simiulate_grasp(x, y, z, z_ori, force, lifting_distance)



# Function simulate_grasp must be defined elsewhere
# Example definition:
# def simulate_grasp(x, y, z):
#     return some_value_based_on_simulation
for i in range(2, 100):
  # Open a CSV file for writing
  with open(f"simulation_results_batch{i}.csv", mode="w", newline="") as file:
      writer = csv.writer(file)
      
      # Write the header
      writer.writerow(["x", "y", "z", "z_ori", "result"])
      
  # Run the simulation 100 times
  for _ in range(100):
      # Generate random inputs for x, y, z
      x = np.random.uniform(-0.075, 0.075)
      y = np.random.uniform(-0.075, 0.075)
      z = np.random.uniform(0.15, 0.2)
      z_ori = np.random.uniform(0, 2 * math.pi)
      # force = np.random.uniform(0, 100)
      force = 75
      lifting_distance = 1.5
  
      gripper1 = gripper()
      # Call the simulate_grasp function
      result = gripper1.simiulate_grasp(x, y, z, z_ori, force, lifting_distance)
      
      # Open a CSV file for writing
      with open(f"simulation_results_batch{i}.csv", mode="a", newline="") as file:
          writer = csv.writer(file)

          # Write the inputs and the result to the CSV
          writer.writerow([x, y, z, z_ori, result])
