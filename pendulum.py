import pybullet as p
import time

GUI = True

q0 = 0.5
dt = 1/240 # pybullet simulation step
if (GUI):
    physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
else:
    physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version

p.setGravity(0, 0, -10)
bodyId = p.loadURDF("./pendulum.urdf")
t = 0
logTime = [t]
logPos = [q0]
maxTime = 20

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.POSITION_CONTROL,
                        targetPosition = q0)

for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)

while t <= maxTime:
    p.stepSimulation()
    t += dt
    logTime.append(t)
    pos = p.getJointState(bodyId, 1)[0]
    logPos.append(pos)
    if (GUI):
        time.sleep(dt)
    
p.disconnect()

import matplotlib.pyplot as plt
plt.plot(logTime, logPos)
plt.grid(True)
plt.show()