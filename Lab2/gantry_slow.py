import vrep
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

import skfuzzy as fuzz
import skfuzzy.control as ctrl

def build_controls():
    global distance, height, speed

    MAX_SPEED = 1
    HALF_MAX_SPEED = 0.5

    FAREST_DISTANCE = 1.4
    HALF_DISTANCE = 0.7

    distance = ctrl.Antecedent(np.arange(-0.5, 2.1, 0.01), 'distance')

    distance['close'] = fuzz.trapmf(distance.universe, [-0.5, -0.5, 0, 0.7])
    distance['semi-close'] = fuzz.trimf(distance.universe, [0, 0.7, 1.4])
    distance['far'] = fuzz.trimf(distance.universe, [0.7, 1.4, 2.1])

    distance['far'].view()

    speed = ctrl.Consequent(np.arange(-0.5, 1.5, 0.01), 'speed')

    speed['low'] = fuzz.trimf(speed.universe, [-0.5, 0, 0.5])
    speed['medium'] = fuzz.trimf(speed.universe, [0, 0.5, 1])
    speed['high'] = fuzz.trimf(speed.universe, [0.5, 1, 1.5])

    speed['high'].view()


def build_rules():
    global acceleration
    rules = []

    rules.append(ctrl.Rule(distance['far'], speed['high']))
    rules.append(ctrl.Rule(distance['semi-close'], speed['medium']))
    rules.append(ctrl.Rule(distance['close'], speed['low']))

    rules[2].view()

    acceleration_ctrl = ctrl.ControlSystem(rules)
    acceleration = ctrl.ControlSystemSimulation(acceleration_ctrl)

    plt.show()


def connect_to_vrep():
    global lf_motor_handle, lb_motor_handle, rb_motor_handle, rf_motor_handle, clientID

    vrep.simxFinish(-1)  # closes all opened connections, in case any prevoius wasnt finished
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit("Could not connect")

    err_code, lf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LF", vrep.simx_opmode_blocking)
    err_code, lb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_LB", vrep.simx_opmode_blocking)
    err_code, rb_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RB", vrep.simx_opmode_blocking)
    err_code, rf_motor_handle = vrep.simxGetObjectHandle(clientID, "WheelJoint_RF", vrep.simx_opmode_blocking)


def set_speed(speed):
    handles = [lf_motor_handle, lb_motor_handle, rb_motor_handle, rf_motor_handle]
    for handle in handles:
        err_code = vrep.simxSetJointTargetVelocity(clientID, handle, speed, vrep.simx_opmode_streaming)


def run_simulation():
    # get handle to the load
    h_min = 1000
    h_max = -1000
    err_code, load_handle = vrep.simxGetObjectHandle(clientID, "Cuboid", vrep.simx_opmode_blocking)

    # get first position of load
    err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_streaming)

    # get positions of load
    t = time.time()
    while True:  # (time.time()-t)<5: # read values for 15 seconds
        err_code, position = vrep.simxGetObjectPosition(clientID, load_handle, -1, vrep.simx_opmode_buffer)

        acceleration.input['distance'] = position[0]
        #acceleration.input['height'] = position[1]
        acceleration.compute()
        speed = acceleration.output['speed']
        set_speed(speed)


        if err_code == 0:
            print("distance = ", position[0], "speed = ", speed)
            # print(position, end="\t")


        if position[1] > h_max:
            h_max = position[1]
            # print("h_max = ", h_max)

        if position[1] < h_min:
            h_min = position[1]
            # print("h_min = ", h_min)


def run_all():
    build_controls()
    build_rules()
    connect_to_vrep()
    run_simulation()


run_all()
