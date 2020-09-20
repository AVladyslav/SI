# import vrep
import sim as vrep
import sys
import time
import numpy as np
from tank import *


vrep.simxFinish(-1)  # closes all opened connections, in case any prevoius wasnt finished
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# create instance of Tank
tank = Tank(clientID)

proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
proximity_sensors_handles = [0] * 8

# get handle to proximity sensors
for i in range(len(proximity_sensors)):
    err_code, proximity_sensors_handles[i] = vrep.simxGetObjectHandle(clientID,
                                                                      "Proximity_sensor_" + proximity_sensors[i],
                                                                      vrep.simx_opmode_blocking)

# read and print values from proximity sensors
# first reading should be done with simx_opmode_streaming, further with simx_opmode_buffer parameter
for sensor_name, sensor_handle in zip(proximity_sensors, proximity_sensors_handles):
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
        clientID, sensor_handle, vrep.simx_opmode_streaming)

tank.forward(5)

# continue reading and printing values from proximity sensors
t = time.time()
while (time.time() - t) < 5:  # read values for 5 seconds
    for sensor_name, sensor_handle in zip(proximity_sensors, proximity_sensors_handles):
        err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handle, vrep.simx_opmode_buffer)
        if err_code == 0:
            print("Proximity_sensor_" + sensor_name, np.linalg.norm(detectedPoint))
    print()
