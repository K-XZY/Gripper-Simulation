import pybullet as p
import pybullet_data
import os
import numpy as np
import sys
import pandas as pd
sys.path.insert(1, 'utils/')
from utils.simulation import sim
from utils.Robot import Robot
from utils.thing import Thing
from utils.pointCloud import getPointCloud
from IPython import embed 

class robotSimulator():
    def __init__(self):
        self.sim = sim()

        self.plane = Thing("plane")

        self.pose = []
        #Create robot object with given arm and gripper
        self.robot = Robot(self.sim.robotParams, self.sim.motionParams["grip_path"])
        self.initButtonVals()
        self.loop()

    def loop(self):
        while(True):
            self.readResetButton()
            self.readToTargetButton()
            self.readGripButton()
            self.readOpenGripperButton()
            self.readTableButton()
            self.readObjectButton()
            self.readPlanPathButton()
            self.readFollowPathButton()
            self.readCameraButton()
            self.readSavePoseButton()
            self.readFollowedSavedPathButton()
            self.readPointCloudButton()
            self.readJointControlButton()
            self.readmultiGraspButton()

            if self.sim.robotParams["camera"]:
                self.robot.simStep()

    def initButtonVals(self):
        self.startButtonVal = 2.0
        self.gripButtonVal = 2.0
        self.openGripperButtonVal = 2.0
        self.resetButtonVal = 2.0
        self.tableButtonVal = 2.0
        self.planButtonVal = 2.0
        self.pathButtonVal = 2.0
        self.cameraButtonVal = 2.0
        self.objButtonVal = 2.0
        self.savePoseButtonVal = 2.0
        self.followSavedPathButtonVal = 2.0
        self.pointCloudButtonVal=2.0
        self.jointControlButtonVal = 2.0
        self.prevVal = 0
        self.multiGraspButtonVal = 2.0

    def readResetButton(self):
        if p.readUserDebugParameter(self.sim.resetButton) >= self.resetButtonVal:
            self.robot.setJointPosition(np.zeros((self.robot.numArmJoints+self.robot.numGripperJoints)))
            if self.tableButtonVal >2.0:
                p.resetBasePositionAndOrientation(self.robot.robot_model,[0,0,self.table.height], self.robot.start_orientation)
            else:
                p.resetBasePositionAndOrientation(self.robot.robot_model, self.robot.start_pos, self.robot.start_orientation)
            self.resetButtonVal = p.readUserDebugParameter(self.sim.resetButton) + 1.0

    def readToTargetButton(self):
        if p.readUserDebugParameter(self.sim.startButton) >= self.startButtonVal:
            #Read data
            data = self.sim.motionParams["pose_target"]

            data = self.robot.createDQObject(data)
            print(f"data = {data}")
            self.robot.moveArmToEETarget(data,0.6)
            self.startButtonVal = p.readUserDebugParameter(self.sim.startButton) + 1.0
            
    def readGripButton(self):
        if p.readUserDebugParameter(self.sim.gripButton) >= self.gripButtonVal:
            self.robot.gripper.grip()
            self.gripButtonVal = p.readUserDebugParameter(self.sim.gripButton) + 1.0

    def readOpenGripperButton(self): 
        if p.readUserDebugParameter(self.sim.openGripperButton) >= self.openGripperButtonVal:
            self.robot.gripper.openGripper()
            self.openGripperButtonVal = p.readUserDebugParameter(self.sim.openGripperButton) + 1.0

    def readTableButton(self):
        if p.readUserDebugParameter(self.sim.tableButton) >= self.tableButtonVal:
            self.table = Thing("table")
            p.resetBasePositionAndOrientation(self.robot.robot_model, [0,0,self.table.height], self.robot.start_orientation)
            self.tableButtonVal = p.readUserDebugParameter(self.sim.tableButton) + 1.0

    def readObjectButton(self):
        if p.readUserDebugParameter(self.sim.objButton) >= self.objButtonVal:
            if self.tableButtonVal > 2.0:
                self.container = Thing(self.sim.simParams["object_name"],onObject=self.table)
            else: 
                self.container = Thing(self.sim.simParams["object_name"],self.sim.simParams["object_position"])
                # print(f"self.container.getPOs: {self.container.getPos()}")
            self.objButtonVal = p.readUserDebugParameter(self.sim.objButton) + 1.0

    def readPlanPathButton(self):
        # if p.readUserDebugParameter(self.sim.planButton) >= self.planButtonVal:
        #     #TODO: add path planning
        #     print("[INFO]: not yet implemented")
        #     self.planButtonVal = p.readUserDebugParameter(self.sim.planButton) + 1.0
        
        if p.readUserDebugParameter(self.sim.planButton) >= self.planButtonVal:
            if hasattr(self, "container"):
                print(f"Getting information from the container...")
                print(f" continer.getInitpos: {self.container.getInitPos()}")
                print(f" continer.getInitOrientaion: {self.container.getInitOrientation()}")
                print(f" continer.getpos: {self.container.getPos()}") 
                pos = self.container.getPos()
                ori = self.container.getInitOrientation()
                data = [0, *pos, *ori]
                print(f"data = {data}")
                data = self.robot.createDQObject(data)
                print(f"data = {data}")
                self.robot.moveArmToEETarget(data,1.6)

            else:
                print("[Error]: No object found")
            self.planButtonVal = p.readUserDebugParameter(self.sim.planButton) + 1.0
            
        # if p.readUserDebugParameter(self.sim.planButton) >= self.planButtonVal:
        #     if hasattr(self, "container"):
        #         # Get the current end-effector position and target object position
        #         current_pos = np.array(self.robot.getPose())
        #         target_pos = np.array(self.container.getPos())

        #         # Calculate a direct path (list of waypoints) from current position to target
        #         num_waypoints = 10  # Define how many points you want in the path
        #         self.path = [current_pos + (target_pos - current_pos) * (i / num_waypoints) for i in range(1, num_waypoints + 1)]

        #         print("[INFO]: Path planned with waypoints.")
        #     else:
        #         print("[ERROR]: No target object found to plan path.")
        
        # self.planButtonVal = p.readUserDebugParameter(self.sim.planButton) + 1.0
        
    def readFollowPathButton(self):
        if p.readUserDebugParameter(self.sim.pathButton) >= self.pathButtonVal:
            #TODO: add follow path
            print("[INFO]: not yet implemented")
            self.pathButtonVal = p.readUserDebugParameter(self.sim.pathButton) + 1.0
        # if p.readUserDebugParameter(self.sim.pathButton) >= self.pathButtonVal:
        #     if hasattr(self, "path") and self.path:
        #         epsilon = 0.01  # Set tolerance value
        #         for waypoint in self.path:
        #             dq_waypoint = self.robot.createDQObject(waypoint)  # Convert to DQ format if needed
        #             self.robot.moveArmToEETarget(dq_waypoint, epsilon)
        #         print("[INFO]: Path followed successfully.")
        #     else:
        #         print("[ERROR]: No path to follow. Please plan a path first.")

        # self.pathButtonVal = p.readUserDebugParameter(self.sim.pathButton) + 1.0

    def readCameraButton(self):
        if p.readUserDebugParameter(self.sim.cameraButton) >= self.cameraButtonVal:
            if hasattr(self,"container"):
                self.sim.getPicture(self.container.getPos())
            else:
                self.sim.getPicture()

            self.cameraButtonVal = p.readUserDebugParameter(self.sim.cameraButton) + 1.0

    def readSavePoseButton(self):
        if p.readUserDebugParameter(self.sim.savePoseButton) >= self.savePoseButtonVal:
            currPose = self.robot.getPose()
            self.pose.append(currPose)
            print("Saved pose: ",currPose)
            self.savePoseButtonVal = p.readUserDebugParameter(self.sim.savePoseButton) + 1.0
    
    def readFollowedSavedPathButton(self):
        if p.readUserDebugParameter(self.sim.followSavedPathButton) >= self.followSavedPathButtonVal:
            self.robot.followPath(self.pose)
            self.followSavedPathButtonVal = p.readUserDebugParameter(self.sim.followSavedPathButton) + 1.0

    def readPointCloudButton(self):
        if p.readUserDebugParameter(self.sim.pointCloudButton) >= self.pointCloudButtonVal:
            if hasattr(self,"container"):
                self.sim.getPointCloud(self.container.getPos(), self.robot.getPandO())
            else:
                self.sim.getPointCloud()
            self.pointCloudButtonVal = p.readUserDebugParameter(self.sim.pointCloudButton) + 1.0

    def readJointControlButton(self):
        if p.readUserDebugParameter(self.sim.jointControlButton) == self.jointControlButtonVal:
            self.sim.addjointControl(self.robot.numArmJoints,self.robot.getJointPosition())
            self.jointControlButtonVal = 1
            self.prevVal = self.sim.getJointControlVal()
        
        if self.jointControlButtonVal == 1:
            targetPos = self.sim.getJointControlVal()
            if self.prevVal != targetPos:
                self.robot.setJointPosition(targetPos)
                self.prevVal = self.sim.getJointControlVal()
    
    def readmultiGraspButton(self):
        if p.readUserDebugParameter(self.sim.multiGraspButton) >= self.multiGraspButtonVal:
            self.robot.multiGraspSimulation()
            self.multiGraspButtonVal = p.readUserDebugParameter(self.sim.multiGraspButton) + 1.0

if __name__ == "__main__":
    main = robotSimulator()

    