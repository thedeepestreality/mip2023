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
maxTime = 6
T = 5
logTime = np.arange(0.0, maxTime, dt)
sz = logTime.size
logPos = np.zeros(sz)
logVel = np.zeros(sz)
logAcc = np.zeros(sz)
logCtr = np.zeros(sz-1)
logPos[0] = q0
logRef = np.zeros(sz)
logRef[0] = q0
logRefd = np.zeros(sz)
logRefdd = np.zeros(sz)

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
kp = 16.18033989  
kv = 31.65059322
# kp = 6.18033989
# kv = 31.57175666
kp = 5005
kv = 100
ki = 0
sum = 0
qd = 0.2

def cubic_interpol(q0, qd, T, t):
    a0 = 0
    a1 = 0
    a2 = 3/T**2
    a3 = -2/T**3
    s = a3*t**3 + a2*t**2 + a1*t + a0
    ds = 3*a3*t**2 + 2*a2*t + a1
    dds = 6*a3*t + 2*a2
    q = q0 + (qd - q0)*s
    dq = (qd - q0)*ds
    ddq = (qd - q0)*dds
    return (q, dq, ddq) if (t <= T) else (qd, 0, 0)

def feedback_lin(pos, vel, posd, veld, accd):
    u = -kp*(pos - posd) -kv*vel
    ctrl = m*L*L*((g/L)*math.sin(pos)+kf/(m*L*L)*vel + u)
    return ctrl
prev_vel = 0
for t in logTime[1:]:
    # pos -= math.pi
    e = pos-qd
    sum += e*dt
    #ctrl = -kp*e - kv*vel - ki*sum
    (posd, veld, accd)  = cubic_interpol(q0, qd, T, t)
    ctrl = feedback_lin(pos, vel, posd, veld, accd)
    logRef[idx] = posd
    logRefd[idx] = veld
    logRefdd[idx] = accd
    p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = 1,
                        controlMode = p.TORQUE_CONTROL,
                        force = ctrl)
    p.stepSimulation()
    pos = p.getJointState(bodyId, 1)[0]
    vel = p.getJointState(bodyId, 1)[1]
    acc = (vel - prev_vel)/dt
    prev_vel = vel
    logPos[idx] = pos
    logVel[idx] = vel
    logAcc[idx] = acc
    logCtr[idx-1] = ctrl
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
plt.subplot(4,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot(logTime, logRef, label = "simRef")
plt.legend()

plt.subplot(4,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.plot(logTime, logRefd, label = "simRefd")
plt.legend()

plt.subplot(4,1,3)
plt.grid(True)
plt.plot(logTime, logAcc, label = "simAcc")
plt.plot(logTime, logRefdd, label = "simRefdd")
plt.legend()

plt.subplot(4,1,4)
plt.grid(True)
plt.plot(logTime[0:-1], logCtr, label = "simCtr")
plt.legend()

plt.show()