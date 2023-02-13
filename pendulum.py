import pybullet as p
import time

dt = 1/240 # pybullet simulation step
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setGravity(0, 0, -10)
bodyId = p.loadURDF("./pendulum.urdf")

while True:
    p.stepSimulation()
    time.sleep(dt)
