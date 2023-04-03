import pybullet as p
import time
import pybullet_data
import numpy as np
from scipy.integrate import odeint
import math
import copy

GUI = False

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

g = 10
m = 1
L = 0.5
kf = 0.1
p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./pendulum.urdf")


q0 = 0.1 # starting position
dt = 1/240 # pybullet simulation step
t = 0
maxTime = 5
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

q0 = p.getJointState(bodyId, 1)[0]

# get rid of the default damping
p.changeDynamics(bodyId, 1, linearDamping = 0)

# let pendulum joint rotate freely
p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
idx = 1
pos = q0
vel = 0
kp = 2
kv = 1
for t in logTime[1:]:
    p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.TORQUE_CONTROL,
                        force = -kp*pos - kv*vel)
    p.stepSimulation()
    pos = p.getJointState(bodyId, 1)[0]
    vel = p.getJointState(bodyId, 1)[1]
    logPos[idx] = pos
    idx += 1
    if (GUI):
        time.sleep(dt)
    
p.disconnect()

def rp(x, t):
    return [x[1], -(g/L)*math.sin(x[0]) -kf/(m*L*L)*x[1] + 0.2/(m*L*L) ]

def rp_lin(x, t):
    return [x[1], -(g/L)*x[0] -kf/(m*L*L)*x[1] ]

theta = odeint(rp, [q0, 0], logTime)
logOdeint = theta[:,0]

def cost(q_exp, q_theor):
    l2 = 0
    sz = len(q_exp)
    linf = abs(q_exp[0] - q_theor[0])
    for i in range(sz):
        err = abs(q_exp[i] - q_theor[i])
        if (err > linf):
            linf = err
        l2 += err**2
    l2 = math.sqrt(l2)
    return (l2, linf)

def symp_euler(fun, x0, TT):
    x1 = copy.copy(x0)
    xx = np.array(x1)
    for i in range(len(TT)-1):
        dt = (TT[i+1] - TT[i])
        x1[1] += fun(x1, 0)[1]*dt
        x1[0] += x1[1]*dt
        xx = np.vstack((xx,x1))
    return xx

(l2_ode, linf_ode) = cost(logPos, logOdeint)

logEuler = symp_euler(rp, [q0, 0], logTime)
logEuler = logEuler[:,0]

(l2_euler, linf_euler) = cost(logPos, logEuler)

logLin = symp_euler(rp_lin, [q0, 0], logTime)
logLin = logLin[:,0]

(l2_lin, linf_lin) = cost(logPos, logLin)

print()
print(f'L2 odeint = {l2_ode}')
print(f'L2 euler = {l2_euler}')
# print(f'L2 lin = {l2_lin}')
print(f'Linf odeint = {linf_ode}')
print(f'Linf euler = {linf_euler}')
# print(f'Linf lin = {linf_lin}')

import matplotlib.pyplot as plt
plt.plot(logTime, logPos, label = "sim")
plt.grid(True)
# plt.plot(logTime, logOdeint, label = "odeint")
# plt.plot(logTime, logEuler, label = "euler")
# plt.plot(logTime, logLin, label = "lin")
plt.legend()
plt.show()