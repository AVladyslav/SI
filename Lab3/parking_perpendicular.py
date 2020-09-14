from tank import Tank
import vrep
import sys
import time
import numpy as np

import skfuzzy as fuzz
import skfuzzy.control as ctrl


def build_controls():
    global right_speed, left_speed, north_west_dist

    right_speed = ctrl.Consequent(np.arange(-12, 12, 0.01), 'right_vel')
    right_speed['low'] = fuzz.trimf(right_speed.universe, [-4, 0, 4])
    right_speed['medium'] = fuzz.trimf(right_speed.universe, [0, 4, 8])
    right_speed['high'] = fuzz.trapmf(right_speed.universe, [9, 10, 12, 12])
    # right_speed['high'].view()

    left_speed = ctrl.Consequent(np.arange(-12, 12, 0.01), 'left_vel')
    left_speed['low'] = fuzz.trimf(left_speed.universe, [-4, 0, 4])
    left_speed['medium'] = fuzz.trimf(left_speed.universe, [0, 2.2, 5.2])
    left_speed['high'] = fuzz.trimf(left_speed.universe, [0, 4, 8])
    # left_speed['high'].view()

    north_west_dist = ctrl.Antecedent(np.arange(0, 4, 0.01), 'nw')
    north_west_dist['very close'] = fuzz.trapmf(north_west_dist.universe, [0, 0, 0.5, 0.6])
    north_west_dist['close'] = fuzz.trapmf(north_west_dist.universe, [0.5, 0.6, 0.9, 1])
    north_west_dist['medium'] = fuzz.trapmf(north_west_dist.universe, [0.9, 1, 2.6, 2.8])
    north_west_dist['far'] = fuzz.trapmf(north_west_dist.universe, [2.6, 2.8, 4, 4])
    # north_west_dist['far'].view()


def build_rules():
    global acceleration

    rule5 = ctrl.Rule(north_west_dist['very close'], left_speed['low'])
    rule4 = ctrl.Rule(north_west_dist['very close'], right_speed['low'])
    rule7 = ctrl.Rule(north_west_dist['close'], left_speed['high'])
    rule3 = ctrl.Rule(north_west_dist['close'], right_speed['medium'])
    rule6 = ctrl.Rule(north_west_dist['medium'], left_speed['medium'])
    rule1 = ctrl.Rule(north_west_dist['medium'], right_speed['high'])
    rule2 = ctrl.Rule(north_west_dist['far'], left_speed['high'])
    rule8 = ctrl.Rule(north_west_dist['far'], right_speed['medium'])

    speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
    acceleration = ctrl.ControlSystemSimulation(speed_ctrl)


def connect_to_vrep():
    global clientID
    vrep.simxFinish(-1)  # closes all opened connections, in case any prevoius wasnt finished
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # start a connection

    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit("Could not connect")


def setup_proximity_sensors():
    global turn, tank, t
    tank = Tank(clientID)
    proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
    proximity_sensors_handles = [0] * 8

    # get handle to proximity sensors
    for i in range(len(proximity_sensors)):
        err_code, proximity_sensors_handles[i] = vrep.simxGetObjectHandle(clientID,
                                                                          "Proximity_sensor_" + proximity_sensors[i],
                                                                          vrep.simx_opmode_blocking)

    # first reading should be done with simx_opmode_streaming, further with simx_opmode_buffer parameter
    for sensor_name, sensor_handle in zip(proximity_sensors, proximity_sensors_handles):
        err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
            clientID, sensor_handle, vrep.simx_opmode_streaming)

    tank.forward(5)
    t = time.time()
    turn = False


def run_simulation():
    global turn
    while (time.time() - t) < 90:
        nw_dist = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 82, vrep.simx_opmode_buffer)[2])
        acceleration.input['nw'] = nw_dist

        if nw_dist > 3.5 and not turn:
            turn = True

        if turn:
            acceleration.compute()
            right_speed = acceleration.output['right_vel']
            left_speed = acceleration.output['left_vel']

            print("nw = ", nw_dist)
            if nw_dist < 0.5:
                print("very close")
            elif nw_dist > 0.6 and nw_dist < 0.9:
                print("close")
            elif nw_dist > 1 and nw_dist < 2.6:
                print("medium")
            elif nw_dist > 2.8:
                print("far")

            tank.rightvelocity = right_speed
            tank.leftvelocity = left_speed
            tank.setVelocity()

        time.sleep(0.2)


def run_all():
    build_controls()
    build_rules()
    connect_to_vrep()
    setup_proximity_sensors()
    run_simulation()


run_all()
