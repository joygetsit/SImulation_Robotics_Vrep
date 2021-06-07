#!/usr/bin/env python3

import sim
import time
import sys
import numpy as np
import math
import matplotlib.pyplot as mlp
PI = math.pi

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print('Connected to remote API server')
else:
    print('Connection to remote API failed')

errorcode, leftmotorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
# print("ErrorCode: %d, Handleleftmotor: %d" % (errorcode, leftmotorHandle))
errorcode, rightmotorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
# print("ErrorCode: %d, Handlerightmotor: %d" % (errorcode, rightmotorHandle))
#
# Now retrieve streaming data (i.e. in a non-blocking fashion):
startTime = time.time()
while time.time() - startTime < 5:
    # Move commands
    errorcode2 = sim.simxSetJointTargetVelocity(clientID, leftmotorHandle, 10, sim.simx_opmode_streaming)
    print(errorcode2)

    errorcode3 = sim.simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.2, sim.simx_opmode_streaming)
    print(errorcode3)

while time.time() - startTime < 7:
    # Stop the motors
    errorcode2 = sim.simxSetJointTargetVelocity(clientID, leftmotorHandle, 0, sim.simx_opmode_streaming)
    print(errorcode2)
    errorcode3 = sim.simxSetJointTargetVelocity(clientID, rightmotorHandle, 0, sim.simx_opmode_streaming)
    print(errorcode3)

sensor_val = np.array([])
sensor_loc = np.array([-90 / 180.0 * PI,
                       -50 / 180.0 * PI,
                       -30 / 180.0 * PI,
                       -10 / 180.0 * PI,
                       10 / 180.0 * PI,
                       30 / 180.0 * PI,
                       50 / 180.0 * PI,
                       90 / 180.0 * PI,
                       ])
errorcode, sr1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_blocking)
print("ErrorCode: %d, sr1: %d" % (errorcode, sr1))
errorcode, sr16 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor16', sim.simx_opmode_blocking)
print("ErrorCode: %d, sr16: %d" % (errorcode, sr16))

# First call of proximity sensor with opmode - streaming
err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
    sim.simxReadProximitySensor(clientID, sr1, sim.simx_opmode_streaming)

# Get Camera Handle
errorcode, cam_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
print("ErrorCode: %d, cam_handle: %d" % (errorcode, cam_handle))
erc, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_streaming)

# Now retrieve streaming data (i.e. in a non-blocking fashion):
startTime = time.time()
while time.time() - startTime < 5:
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
        sim.simxReadProximitySensor(clientID, sr1, sim.simx_opmode_buffer) # Subsequent calls with buffer
    erc, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_buffer)
    # im=np.array(image, dtype=np.uint8)
    # print(im.shape)
    # im.resize([resolution[0], resolution[1], 3])
    # print(im.shape)
    # mlp.imshow(im)

# Stop  the simulation
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)

print('Program ended')