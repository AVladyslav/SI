import vrep
import sys
import time
import numpy as np

vrep.simxFinish(-1)  # closes all opened connections, in case any prevoius wasnt finished
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# get handles to robot drivers
err_code, lf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LF", vrep.simx_opmode_blocking)
err_code, lb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LB", vrep.simx_opmode_blocking)
err_code, rb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RB", vrep.simx_opmode_blocking)
err_code, rf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RF", vrep.simx_opmode_blocking)

# Turn on the drives
err_code = vrep.simxSetJointTargetVelocity(clientID, lb_motor_handle, 1.0, vrep.simx_opmode_streaming)
err_code = vrep.simxSetJointTargetVelocity(clientID, rb_motor_handle, 1.0, vrep.simx_opmode_streaming)
err_code = vrep.simxSetJointTargetVelocity(clientID, lf_motor_handle, 1.0, vrep.simx_opmode_streaming)
err_code = vrep.simxSetJointTargetVelocity(clientID, rf_motor_handle, 1.0, vrep.simx_opmode_streaming)

# get handle to the load
err_code, load_handle = vrep.simxGetObjectHandle(clientID, "Load", vrep.simx_opmode_blocking)

# get first position of load
err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_streaming)

# get positions of load
t = time.time()
while (time.time() - t) < 15:  # read values for 15 seconds
    err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_buffer)
    if err_code == 0:
        print(position)
