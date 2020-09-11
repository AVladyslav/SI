import vrep
import sys
import time
import numpy as np

import skfuzzy as fuzz
import skfuzzy.control as ctrl

DESTINATION_POINT = 0.1

# # Triangular membership function
#
# distance = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'distance')
# speed = ctrl.Consequent(np.arange(0, 1.5, 0.1), 'speed')
#
# distance['close'] = fuzz.trimf(distance.universe, [0, 0, 0.6])
# distance['medium'] = fuzz.trimf(distance.universe, [0.4, 0.75, 1.1])
# distance['far'] = fuzz.trimf(distance.universe, [0.9, 1.5, 1.5])
#
# distance.view()
#
# speed['low'] = fuzz.trimf(speed.universe, [-1000, -500, 0.5])
# speed['medium'] = fuzz.trimf(speed.universe, [0.2, 0.7, 1.2])
# speed['high'] = fuzz.trimf(speed.universe, [0.7, 1, 1.3])
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
# control.input['distance'] = 1.3  # random value
# control.compute()
# print(control.output['speed'])


# # Trapezoidal membership function
#
# distance = ctrl.Antecedent(np.arange(0, 1.5, 0.1), 'distance')
# speed = ctrl.Consequent(np.arange(0, 1.5, 0.1), 'speed')
#
# distance['close'] = fuzz.trapmf(distance.universe, [0, 0, 0.3, 0.6])
# distance['medium'] = fuzz.trapmf(distance.universe, [0.3, 0.6, 0.9, 1.2])
# distance['far'] = fuzz.trapmf(distance.universe, [0.9, 1.2, 1.5, 2.0])
#
# distance.view()
#
# speed['low'] = fuzz.trapmf(speed.universe, [-2, -1, 0, 0.2])
# speed['medium'] = fuzz.trapmf(speed.universe, [0.1, 0.5, 0.7, 0.9])
# speed['high'] = fuzz.trapmf(speed.universe, [0.7, 0.9, 1, 1])
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
# control.input['distance'] = 0.1  # random value
# control.compute()
# print(control.output['speed'])


# Sinusoidal membership function

distance = ctrl.Antecedent(np.arange(0, 1.5, 0.01), 'distance')
speed = ctrl.Consequent(np.arange(-0.1, 1.01, 0.01), 'speed')

distance['close'] = fuzz.sigmf(distance.universe, 0.3, -15)
distance['medium'] = fuzz.gaussmf(distance.universe, 0.7, 0.2)
distance['far'] = fuzz.sigmf(distance.universe, 1.2, 15)

distance.view()

speed['low'] = fuzz.sigmf(speed.universe, 0.2, -20)
speed['medium'] = fuzz.gaussmf(speed.universe, 0.5, 0.15)
speed['high'] = fuzz.sigmf(speed.universe, 0.8, 20)

speed.view()

rule1 = ctrl.Rule(distance['close'], speed['low'])
rule2 = ctrl.Rule(distance['medium'], speed['medium'])
rule3 = ctrl.Rule(distance['far'], speed['high'])

control_system = ctrl.ControlSystem([rule1, rule2, rule3])
control = ctrl.ControlSystemSimulation(control_system)

control.input['distance'] = 0.1  # random value
control.compute()
print(control.output['speed'])


def set_speed(velocity):
    vrep.simxSetJointTargetVelocity(clientID, lb_motor_handle, velocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rb_motor_handle, velocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, lf_motor_handle, velocity, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rf_motor_handle, velocity, vrep.simx_opmode_streaming)


vrep.simxFinish(-1)  # closes all opened connections, in case any previous wasn't finished
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

# get handles to robot drivers
lf_err_code, lf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LF", vrep.simx_opmode_blocking)
lb_err_code, lb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LB", vrep.simx_opmode_blocking)
rb_err_code, rb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RB", vrep.simx_opmode_blocking)
rf_err_code, rf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RF", vrep.simx_opmode_blocking)

# Turn on the drives
# err_code = vrep.simxSetJointTargetVelocity(clientID, lb_motor_handle, 1.0, vrep.simx_opmode_streaming)
# err_code = vrep.simxSetJointTargetVelocity(clientID, rb_motor_handle, 1.0, vrep.simx_opmode_streaming)
# err_code = vrep.simxSetJointTargetVelocity(clientID, lf_motor_handle, 1.0, vrep.simx_opmode_streaming)
# err_code = vrep.simxSetJointTargetVelocity(clientID, rf_motor_handle, 1.0, vrep.simx_opmode_streaming)

# get handle to the load
load_err_code, load_handle = vrep.simxGetObjectHandle(clientID, "Load", vrep.simx_opmode_blocking)

# get first position of load
err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_streaming)

# get positions of load
t = time.time()
while True:  # read values for 15 seconds
    err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_buffer)
    if err_code == 0:
        distance = position[0] - DESTINATION_POINT
        control.input['distance'] = distance
        control.compute()
        speed = control.output['speed']
        set_speed(speed)
        print('Position =', position[0], '\t\tSpeed =', speed)
