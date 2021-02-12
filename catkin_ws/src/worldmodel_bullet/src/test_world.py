import pybullet as p
import pybullet_data
import rospkg
import os

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
path = r.get_path('worldmodel_bullet')

urdf_path = os.path.join(path, 'urdf/')

map_path = r.get_path('worldmodel_maps')
map_path = os.path.join(map_path,'models/fyp_tracks/')

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
# p.setAdditionalSearchPath(os.path.join(path,'urdf/')) #used by loadURDF
p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
planeId = p.loadSDF(os.path.join(map_path,'track1/track1.sdf'))
carStartPos = [0,0,1]
carStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF(os.path.join(urdf_path, "em_3905.urdf"),carStartPos, carStartOrientation)
p.stepSimulation()
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
input()

for i in range(1000):
    p.stepSimulation()

input()
p.disconnect()