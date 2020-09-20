import vrep
import sys
import time
import numpy as np
from tank import *

import skfuzzy as fuzz
import skfuzzy.control as ctrl

# vrep.simxFinish(-1) # closes all opened connections, in case any prevoius wasnt finished
# clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

# if clientID!=-1:
#     print ("Connected to remote API server")
# else:
#     print("Not connected to remote API server")
#     sys.exit("Could not connect")

# #create instance of Tank
# tank=Tank(clientID)

# # get handle to proximity sensor
# err_code,ps_handle = vrep.simxGetObjectHandle(clientID,"Proximity_sensor", vrep.simx_opmode_blocking)

# t = time.time()
# while (time.time()-t)<30: # 10 seconds of communitation
#     #read values from proximity sensor
#     err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,ps_handle,vrep.simx_opmode_blocking)
#     distance = np.linalg.norm(detectedPoint)
#     print(distance)
# ##     Punktowo
# #     if distance <= 2:
# #         print("Stop")
# #         tank.stop()
# #     elif distance <= 4:
# #         print("Slow down")
# #         tank.forward(5)
# #     else:
# #         tank.forward(10)
#     if distance <= 2:
#         print("Stop")
#         tank.stop()
#     elif distance <= 4:
#         speed = (distance - 2) / 2 * 10
#         print("New speed = " + str(speed))
#         tank.forward(speed)
#     else:
#         tank.forward(10)

#     # avoid collision

# vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot) # stop the simulation in vrep


MAX_SPEED = 15
HALF_MAX_SPEED = 7.5

FAREST_DISTANCE = 7
HALF_DISTANCE = 5

distance = ctrl.Antecedent(np.arange(0, 10, 1), 'distance')

distance['close'] = fuzz.trapmf(distance.universe, [-1, 0, 1, 2])
distance['semi-close'] = fuzz.trimf(distance.universe, [2, 3, 5])
distance['far'] = fuzz.trimf(distance.universe, [4, 7, 7])

distance['far'].view()

speed = ctrl.Consequent(np.arange(0, 15, 1), 'speed')
# speed.automf(3)
# speed['average'].view()

speed['low'] = fuzz.trimf(speed.universe, [-1000, -500, 2])
speed['medium'] = fuzz.trimf(speed.universe, [2, HALF_MAX_SPEED, MAX_SPEED])
speed['high'] = fuzz.trimf(speed.universe, [HALF_MAX_SPEED, MAX_SPEED, MAX_SPEED])

speed['high'].view()

rule1 = ctrl.Rule(distance['far'], speed['high'])
rule2 = ctrl.Rule(distance['semi-close'], speed['medium'])
rule3 = ctrl.Rule(distance['close'], speed['low'])
# rule1 = ctrl.Rule(distance['very_close'], speed['stop'])

rule3.view()

acceleration_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
acceleration = ctrl.ControlSystemSimulation(acceleration_ctrl)

acceleration.input['distance'] = 0.5
acceleration.compute()

print(acceleration.output['speed'])
distance.view(sim=acceleration)

vrep.simxFinish(-1)  # closes all opened connections, in case any prevoius wasnt finished
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
while (time.time() - t) < 60:  # 10 seconds of communitation
    # read values from proximity sensor
    err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
        vrep.simxReadProximitySensor(clientID, ps_handle, vrep.simx_opmode_blocking)
    distance = np.linalg.norm(detectedPoint)
    #     print('Dstance = ' + str(distance))
    # Jezeli daleko to jedz szybko
    # Jezeli srednio daleko to powoli zwalniaj
    # Jezeli blisko to hamuj do zera

    # avoid collision
    #     if(distance <= 1):
    #         speed = 0
    #     else:
    acceleration.input['distance'] = distance
    acceleration.compute()
    speed = acceleration.output['speed']

    tank.forward(speed)
    print('Dstance = ' + str(distance) + "\t\t\tNew speed = " + str(speed))

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)  # stop the simulation in vrep
