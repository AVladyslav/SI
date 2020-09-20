# import vrep
import sim as vrep
import sys
import time 
import numpy as np
from tank import *

import skfuzzy as fuzz
import skfuzzy.control as ctrl


vrep.simxFinish(-1)  # closes all opened connections, in case any previous wasn't finished
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# # create instance of Tank
# tank = Tank(clientID)

# # get handle to proximity sensor
# err_code, ps_handle = vrep.simxGetObjectHandle(clientID, "Proximity_sensor", vrep.simx_opmode_blocking)

# t = time.time()
# while (time.time()-t) < 60:  # 30 seconds of communication
#     # read values from proximity sensor
#     err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector =\
#         vrep.simxReadProximitySensor(clientID, ps_handle, vrep.simx_opmode_streaming)
#     distance = np.linalg.norm(detectedPoint)

#     # if distance > 3.5:
#     #     tank.forward(10)
#     # elif distance > 1:
#     #     tank.forward(5)
#     # else:
#     #     tank.stop()

#     if distance > 5:
#         tank.forward(10)
#     elif distance > 1:
#         speed = distance * 2
#         tank.forward(speed)
#         print("New speed = ", speed)
#     else:
#         tank.stop()
#     # print(distance)
#     # avoid collision

# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)  # stop the simulation in vrep

# distance = ctrl.Antecedent(np.arange(0, 10, 1), 'distance')
# speed = ctrl.Consequent(np.arange(0, 13, 1), 'speed')
#
# distance['close'] = fuzz.trapmf(distance.universe, [0, 0, 1, 3])
# distance['medium'] = fuzz.trimf(distance.universe, [2, 4, 5])
# distance['far'] = fuzz.trapmf(distance.universe, [4, 7, 10, 10])
#
# distance.view()
#
# speed['low'] = fuzz.trimf(speed.universe, [0, 0, 2])
# speed['medium'] = fuzz.trimf(speed.universe, [0, 4, 8])
# speed['high'] = fuzz.trapmf(speed.universe, [4, 9, 15, 15])
#
# speed.view()
#
# rule1 = ctrl.Rule(distance['close'], speed['low'])
# rule2 = ctrl.Rule(distance['medium'], speed['medium'])
# rule3 = ctrl.Rule(distance['far'], speed['high'])
#
# control_system = ctrl.ControlSystem([rule1, rule2, rule3])
# control = ctrl.ControlSystemSimulation(control_system)
#
# control.input['distance'] = 3  # random value
# control.compute()
# print(control.output['speed'])


# Model "agresywny"

distance = ctrl.Antecedent(np.arange(0, 10, 1), 'distance')
speed = ctrl.Consequent(np.arange(0, 13, 1), 'speed')

distance['close'] = fuzz.trapmf(distance.universe, [0, 0, 1, 2])
distance['medium'] = fuzz.trimf(distance.universe, [2, 3.5, 5])
distance['far'] = fuzz.trapmf(distance.universe, [5, 6, 10, 10])

distance.view()

speed['low'] = fuzz.trimf(speed.universe, [0, 0, 2])
speed['medium'] = fuzz.trimf(speed.universe, [0, 4, 8])
speed['high'] = fuzz.trapmf(speed.universe, [4, 9, 15, 15])

speed.view()

rule1 = ctrl.Rule(distance['close'], speed['low'])
rule2 = ctrl.Rule(distance['medium'], speed['medium'])
rule3 = ctrl.Rule(distance['far'], speed['high'])

control_system = ctrl.ControlSystem([rule1, rule2, rule3])
control = ctrl.ControlSystemSimulation(control_system)

control.input['distance'] = 3  # random value
control.compute()
print(control.output['speed'])

# vrep.simxFinish(-1)  # closes all opened connections, in case any previous wasn't finished
# clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

# if clientID != -1:
#     print("Connected to remote API server")
# else:
#     print("Not connected to remote API server")
#     sys.exit("Could not connect")

# create instance of Tank
tank = Tank(clientID)

# get handle to proximity sensor
err_code, ps_handle = vrep.simxGetObjectHandle(clientID, "Proximity_sensor", vrep.simx_opmode_blocking)

t = time.time()
while True:  # 30 seconds of communication
    # read values from proximity sensor
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector =\
        vrep.simxReadProximitySensor(clientID, ps_handle, vrep.simx_opmode_streaming)
    distance = np.linalg.norm(detectedPoint)

    control.input['distance'] = distance
    control.compute()
    speed = control.output['speed']
    tank.forward(speed)
    # print('Dist = ', distance, '\t\tSpeed = ', speed)
    print(tank.readVelocity())
    # avoid collision

# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)  # stop the simulation in vrep
