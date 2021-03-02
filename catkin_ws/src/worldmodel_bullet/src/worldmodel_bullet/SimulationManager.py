import pybullet as p
import pybullet_data
from pathlib import Path
# import rospkg
import os
import time
import pkgutil
import numpy as np
import cv2
import sys

SUPPRESS_STDOUT = True
SUPPRESS_STDERR = True

class SimulationManager:

    def __init__(self, render_mode):
        # self.ros_root = rospkg.get_ros_root()
        # self.rospkg = rospkg.RosPack()
        # self.pkg_path = self.rospkg.get_path('worldmodel_bullet')
        self.pkg_path = os.path.join(Path(__file__).resolve().parent, '../../')
        self.tracks_path = os.path.join(self.pkg_path, 'tracks/')
        self.render_mode = render_mode
        
        if render_mode=='human':
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
            egl = pkgutil.get_loader('eglRenderer')
            self.plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            print("plugin=", self.plugin)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            # p.setPhysicsEngineParameter(enableFileCaching=0)

        p.resetSimulation()
        p.setGravity(0,0,-9.81)
        p.setRealTimeSimulation(0)

        self.bodies = dict()
    
    def get_track_paths(self, track_name):

        field_path = os.path.join(self.tracks_path, f'{track_name}_field.obj')
        lines_path = os.path.join(self.tracks_path, f'{track_name}_lines.obj')
        road_path = os.path.join(self.tracks_path, f'{track_name}_road.obj')
        return field_path, lines_path, road_path

    def spawn_track(self, track_name='track1', field_friction=50, road_friction=150, lines_friction=100):
        field_path, lines_path, road_path = self.get_track_paths(track_name)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=field_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=field_path, meshScale=1.0)
        self.field_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=lines_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=lines_path, meshScale=1.0)
        self.lines_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=road_path, meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        visual_id = p.createVisualShape(p.GEOM_MESH, fileName=road_path, meshScale=1.0)
        self.road_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

        p.changeDynamics(self.field_id, -1, lateralFriction=field_friction)
        p.changeDynamics(self.lines_id, -1, lateralFriction=road_friction)
        p.changeDynamics(self.road_id, -1, lateralFriction=lines_friction)
    
    def register_body(self, body_name, body_id):
        self.bodies[body_name] = body_id
    
    def step_simulation(self):
        p.stepSimulation()
    
    def reset_simulation(self):
        p.resetSimulation()
        p.setGravity(0,0,-9.81)
        p.setRealTimeSimulation(0)
    
    def close(self):
        p.disconnect()

class SimulatedCar:

    def __init__(self, start_position=[0,0,0], start_orientation=[0,0,0], render_mode='headless'):
        # self.ros_root = rospkg.get_ros_root()
        # self.rospkg = rospkg.RosPack()
        # self.pkg_path = self.rospkg.get_path('worldmodel_bullet')
        self.pkg_path = os.path.join(Path(__file__).resolve().parent, '../../')
        self.urdf_path = os.path.join(self.pkg_path, 'urdf/')

        self.render_mode = render_mode
        start_orientation = p.getQuaternionFromEuler(start_orientation)
        
        self.car = p.loadURDF(os.path.join(self.urdf_path, 'f10_racecar/racecar_differential.urdf'), start_position, start_orientation)

        for wheel in range(p.getNumJoints(self.car)):
            # print("joint[",wheel,"]=", p.getJointInfo(self.car,wheel))
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

        self.hokuyo_joint=4
        self.zed_camera_joint = 5

        # self.targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
        # self.maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
        # self.steeringSlider = p.addUserDebugParameter("steering",-1,1,0)
    
    def set_speed(self, wheel_vel=0, max_force=10):
        
        for wheel in self.wheels:
            p.setJointMotorControl2(self.car,wheel,p.VELOCITY_CONTROL,targetVelocity=wheel_vel,force=max_force)
    
    def set_steering(self, steering_angle=0):

        for steer in self.steering:
            p.setJointMotorControl2(self.car,steer,p.POSITION_CONTROL,targetPosition=-steering_angle)
    
    def get_position(self):

        pos, _ = p.getBasePositionAndOrientation(self.car)

        return pos
    
    def get_orientation(self, quaternion=False):

        _, orientation = p.getBasePositionAndOrientation(self.car)
        
        if not quaternion:
            orientation =  p.getEulerFromQuaternion(orientation)
        
        return orientation
    
    def get_image(self, image_width=640, image_height=640):

        camTargetPos, orientation = p.getBasePositionAndOrientation(self.car)
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        roll = -(roll / np.pi * 180)
        pitch = -(pitch / np.pi * 180)
        yaw = yaw / np.pi * 180
        camDistance = 1.5
        yaw = yaw - 90
        pitch = -90 + pitch
        
        yaw = self.angle_protection(yaw)
        pitch = self.angle_protection(pitch)
        upAxisIndex = 2
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
                                                     upAxisIndex)
        projectionMatrix = p.computeProjectionMatrixFOV(103, 1, 0.1, 100)

        if self.render_mode=='human':
            img = p.getCameraImage(image_width, image_height, 
                                    viewMatrix=viewMatrix,
                                    projectionMatrix=projectionMatrix,
                                    shadow=1,
                                    lightDirection=[1, 1, 1],
                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)
        else:
            img = img = p.getCameraImage(image_width, image_height,
                                    viewMatrix=viewMatrix,
                                    projectionMatrix=projectionMatrix,
                                    shadow=1,
                                    lightDirection=[1, 1, 1])
        
        self.img = img[2]

        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGBA2RGB)

        self.img = self.img.astype(np.float32)
        self.img /= 255

        return self.img
    
    def angle_protection(self, angle):
        if angle < -180:
            angle = angle + 360
        if angle > 180:
            angle = angle - 360
        
        return angle