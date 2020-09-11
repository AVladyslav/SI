import vrep 
import sys
import time 
import numpy as np
from tank import *


vrep.simxFinish(-1)  # closes all opened connections, in case any previous wasn't finished
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# create instance of Tank
tank = Tank(clientID)

# get handle to proximity sensor
err_code, ps_handle = vrep.simxGetObjectHandle(clientID, "Proximity_sensor", vrep.simx_opmode_blocking)

t = time.time()
while (time.time()-t) < 60:  # 30 seconds of communication
    # read values from proximity sensor
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector =\
        vrep.simxReadProximitySensor(clientID, ps_handle, vrep.simx_opmode_streaming)
    distance = np.linalg.norm(detectedPoint)

    # if distance > 3.5:
    #     tank.forward(10)
    # elif distance > 1:
    #     tank.forward(5)
    # else:
    #     tank.stop()

    if distance > 5:
        tank.forward(10)
    elif distance > 3:
        speed = (distance - 2) / 2 * 10
        print("New speed = " + str(speed))
        tank.forward(speed)
    else:
        tank.stop()
    # print(distance)
    # avoid collision

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)  # stop the simulation in vrep
