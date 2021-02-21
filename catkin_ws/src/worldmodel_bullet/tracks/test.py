import pybullet as p
import pybullet_data
import time

UPDATE_RATE = 24

TRACK_NUM = 20

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=f'./track{TRACK_NUM}_road.obj', meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
visual_id = p.createVisualShape(p.GEOM_MESH, fileName=f'./track{TRACK_NUM}_road.obj', meshScale=1.0)
road_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=f'./track{TRACK_NUM}_lines.obj', meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
visual_id = p.createVisualShape(p.GEOM_MESH, fileName=f'./track{TRACK_NUM}_lines.obj', meshScale=1.0)
road_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

collision_id = p.createCollisionShape(p.GEOM_MESH,fileName=f'./track{TRACK_NUM}_field.obj', meshScale=1.0, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
visual_id = p.createVisualShape(p.GEOM_MESH, fileName=f'./track{TRACK_NUM}_field.obj', meshScale=1.0)
road_id = p.createMultiBody(baseCollisionShapeIndex=collision_id, baseVisualShapeIndex=visual_id)

cubeStartPos = [0,0,5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
p.stepSimulation()
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
input()

step = 0
for i in range(1000):
    p.stepSimulation()
    step += 1
    if step % UPDATE_RATE == 0:
        time.sleep(0.1)

input()
p.disconnect()
