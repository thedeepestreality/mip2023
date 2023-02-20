import pybullet as p
import time
import pybullet_data
import numpy as np

GUI = True

if (GUI):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.2, 
        cameraYaw=90, 
        cameraPitch=0, 
        cameraTargetPosition=[0, 0, 1]
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.addUserDebugLine([0,0,0],[1,0,0],[1,0,0],4)
    p.addUserDebugLine([0,0,0],[0,1,0],[0,1,0],4)
    p.addUserDebugLine([0,0,0],[0,0,1],[0,0,1],4)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeID = p.loadURDF('plane.urdf')
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -10)
bodyId = p.loadURDF("./pendulum.urdf")

q0 = 0.5 # starting position
dt = 1/240 # pybullet simulation step
t = 0
maxTime = 20
logTime = np.arange(0.0, maxTime, dt)
sz = logTime.size
logPos = np.zeros(sz)
logPos[0] = q0

# go to the starting position
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.POSITION_CONTROL,
                        targetPosition = q0)
for _ in range(1000):
    p.stepSimulation()

# make constant oscillations
p.changeDynamics(bodyId, 1, linearDamping = 0)

# let pendulum joint rotate freely
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
idx = 1
for t in logTime[1:]:
    p.stepSimulation()
    pos = p.getJointState(bodyId, 1)[0]
    logPos[idx] = pos
    idx += 1
    if (GUI):
        time.sleep(dt)
    
p.disconnect()

import matplotlib.pyplot as plt
plt.plot(logTime, logPos)
plt.grid(True)
plt.show()