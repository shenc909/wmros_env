# from p_utils import bullet_client as bc
import pybullet as p
import pybullet_data
import rospkg
import os
import time
import numpy as np

import cv2

import pkgutil

class TrackManager:

    def __init__(self):
        self.ros_root = rospkg.get_ros_root()
        self.rospkg = rospkg.RosPack()
        self.pkg_path = self.rospkg.get_path('worldmodel_bullet')
        self.urdf_path = os.path.join(self.pkg_path, 'urdf/')
        self.tracks_path = os.path.join(self.pkg_path, 'tracks/')
        p.connect(p.GUI)
        p.resetSimulation()
        p.setGravity(0,0,-9.81)
        p.setRealTimeSimulation(0)
        # egl = pkgutil.get_loader('eglRenderer')
        # self.plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        # print("plugin=", self.plugin)

        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        

    def get_track_paths(self, track_num):

        field_path = os.path.join(self.tracks_path, f'track{track_num}_field.obj')
        lines_path = os.path.join(self.tracks_path, f'track{track_num}_lines.obj')
        road_path = os.path.join(self.tracks_path, f'track{track_num}_road.obj')
        return field_path, lines_path, road_path

    def spawn_track(self, track_num):
        field_path, lines_path, road_path = self.get_track_paths(track_num)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=field_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=field_path, meshScale=1.0)
        self.field_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=lines_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=lines_path, meshScale=1.0)
        self.lines_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=road_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=road_path, meshScale=1.0)
        self.road_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        p.changeDynamics(self.field_id, -1, lateralFriction=50)
        p.changeDynamics(self.lines_id, -1, lateralFriction=50)
        p.changeDynamics(self.road_id, -1, lateralFriction=50)
    
    def spawn_vehicle(self, start_position=[0,0,0], start_orientation=[0,0,0]):
        start_orientation = p.getQuaternionFromEuler(start_orientation)
        self.car = p.loadURDF(os.path.join(self.urdf_path, 'f10_racecar/racecar_differential.urdf'), start_position, start_orientation)

        for wheel in range(p.getNumJoints(self.car)):
            print("joint[",wheel,"]=", p.getJointInfo(self.car,wheel))
            p.setJointMotorControl2(self.car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
            p.getJointInfo(self.car,wheel)

        self.wheels = [8,15]
        # self.wheels = [12, 14]
        print("----------------")

        #p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
        c = p.createConstraint(self.car,9,self.car,11,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=1, maxForce=10000)

        c = p.createConstraint(self.car,10,self.car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, maxForce=10000)

        c = p.createConstraint(self.car,9,self.car,13,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, maxForce=10000)

        c = p.createConstraint(self.car,16,self.car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=1, maxForce=10000)


        c = p.createConstraint(self.car,16,self.car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, maxForce=10000)

        c = p.createConstraint(self.car,17,self.car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, maxForce=10000)

        c = p.createConstraint(self.car,1,self.car,18,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15, maxForce=10000)
        c = p.createConstraint(self.car,3,self.car,19,jointType=p.JOINT_GEAR,jointAxis =[0,1,0],parentFramePosition=[0,0,0],childFramePosition=[0,0,0])
        p.changeConstraint(c,gearRatio=-1, gearAuxLink = 15,maxForce=10000)


        self.steering = [0,2]

        hokuyo_joint=4
        zed_camera_joint = 5

        self.targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
        self.maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
        self.steeringSlider = p.addUserDebugParameter("steering",-1,1,0)

        self.camera_joint = 20

    def spawn_other_car(self, start_position=[0,0,0], start_orientation=[0,0,0]):
        start_orientation = p.getQuaternionFromEuler(start_orientation)
        self.car = p.loadURDF(os.path.join(self.urdf_path, 'em_3905.urdf'), start_position, start_orientation)

        for wheel in range(p.getNumJoints(self.car)):
            print("joint[",wheel,"]=", p.getJointInfo(self.car,wheel))
            p.setJointMotorControl2(self.car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)
            p.getJointInfo(self.car,wheel)
        
        self.wheels = [12, 14] # 7, 10

        self.steering = [6, 9]

        self.shocks = [5, 8, 11, 13]

        # for shock in self.shocks:
            # p.setJointMotorControl2(self.car, shock, p.POSITION_CONTROL, targetPosition=-0.0042)#, positionGain=297.8719, velocityGain=-72.4832)
        
        self.targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
        self.maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
        self.steeringSlider = p.addUserDebugParameter("steering",-1,1,0)

        

def main():

    tm = TrackManager()
    tm.spawn_track(7)

    tm.spawn_vehicle(start_position=[0,0,3])
    # tm.spawn_other_car(start_position=[0,0,3])

    p.stepSimulation()
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    # print(cubePos,cubeOrn)
    input()

    for i in range(500):
        p.stepSimulation()
        # time.sleep(0.01)

    input()

    while(True):
        # maxForce = p.readUserDebugParameter(tm.maxForceSlider)
        # targetVelocity = p.readUserDebugParameter(tm.targetVelocitySlider)
        # steeringAngle = p.readUserDebugParameter(tm.steeringSlider)

        for wheel in tm.wheels:
            p.setJointMotorControl2(tm.car,wheel,p.VELOCITY_CONTROL,targetVelocity=10,force=20)
            
        for steer in tm.steering:
            p.setJointMotorControl2(tm.car,steer,p.POSITION_CONTROL,targetPosition=-0.3)
        
        # ls = p.getLinkState(tm.car,tm.camera_joint, computeForwardKinematics=True)
        # camPos = ls[0]
        # camOrn = ls[1]
        # camMat = p.getMatrixFromQuaternion(camOrn)
        # upVector = [0,0,1]
        # forwardVec = [camMat[0],camMat[3],camMat[6]]
        # #sideVec =  [camMat[1],camMat[4],camMat[7]]
        # camUpVec =  [camMat[2],camMat[5],camMat[8]]
        # camTarget = [camPos[0]+forwardVec[0]*10,camPos[1]+forwardVec[1]*10,camPos[2]+forwardVec[2]*10]
        # camUpTarget = [camPos[0]+camUpVec[0],camPos[1]+camUpVec[1],camPos[2]+camUpVec[2]]
        # viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
        # projMat = camInfo[3]
		#p.getCameraImage(320,200,viewMatrix=viewMat,projectionMatrix=projMat, flags=p.ER_NO_SEGMENTATION_MASK, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        # img = p.getCameraImage(64,64,viewMatrix=viewMat,projectionMatrix=projMat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        camTargetPos, orientation = p.getBasePositionAndOrientation(tm.car)
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        # print(roll,pitch,yaw)
        roll = roll / np.pi * 180
        pitch = pitch / np.pi * 180
        yaw = yaw / np.pi * 180
        camDistance = 1.5
        yaw = yaw - 90
        pitch = -90 + pitch
        # roll = 0
        upAxisIndex = 2
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
                                                     upAxisIndex)
        projectionMatrix = [
            1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0, 0.0, 0.0,
            -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0
        ]

        projectionMatrix = p.computeProjectionMatrixFOV(103, 1, 0.1, 100)

        start = time.time()
        # img = p.getCameraImage(320,
        #                             320,
        #                             viewMatrix=viewMatrix,
        #                             projectionMatrix=projectionMatrix,
        #                             shadow=1,
        #                             lightDirection=[1, 1, 1])
        
        img = p.getCameraImage(320,320,viewMatrix=viewMatrix,projectionMatrix=projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # print(img[2])

        cv2.imshow('test', cv2.cvtColor(img[2], cv2.COLOR_RGBA2BGRA))
        cv2.waitKey(1)
        
        p.stepSimulation()
        # time.sleep(0.01)

    input()
    p.disconnect()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()