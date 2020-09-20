from tank import Tank
# import vrep
import sim as vrep
import sys
import time
import numpy as np

import skfuzzy as fuzz
import skfuzzy.control as ctrl


def build_controls():
    global ne, sw, nw, first_ne, first_se, searching_speed, right_speed, left_vel, second_ne, second_sw

    sw = ctrl.Antecedent(np.arange(0, 4, 0.01), 'sw')
    sw['close'] = fuzz.trapmf(sw.universe, [0, 0, 1, 1.2])
    sw['medium'] = fuzz.trapmf(sw.universe, [0.9, 1.2, 2.3, 2.6])
    sw['far'] = fuzz.trapmf(sw.universe, [2.3, 2.6, 4, 4])
    # sw['close'].view()

    ne = ctrl.Antecedent(np.arange(0, 4, 0.01), 'ne')
    ne['slow'] = fuzz.trapmf(ne.universe, [0, 0, 0.8, 0.9])
    ne['medium'] = fuzz.trapmf(ne.universe, [0.8, 0.9, 1, 1.3])
    ne['far'] = fuzz.trapmf(ne.universe, [1.2, 1.3, 4, 4])
    # ne['slow'].view()

    nw = ctrl.Antecedent(np.arange(0, 4, 0.01), 'nw')
    nw['slow'] = fuzz.trapmf(nw.universe, [0, 0, 0.8, 1])
    nw['far'] = fuzz.trapmf(nw.universe, [0.99, 1.5, 4, 4])
    # nw['slow'].view()

    first_ne = ctrl.Antecedent(np.arange(0, 4, 0.01), 'first_ne')
    first_ne['far'] = fuzz.trapmf(first_ne.universe, [1.29, 2, 4, 4])
    first_ne['close'] = fuzz.trapmf(first_ne.universe, [0, 0, 1.29, 2])
    # first_ne['close'].view()

    first_se = ctrl.Antecedent(np.arange(0, 4, 0.01), 'first_se')
    first_se['far'] = fuzz.trapmf(first_se.universe, [2, 3.1, 4, 4])
    first_se['close'] = fuzz.trapmf(first_se.universe, [0, 0, 2, 3.1])
    # first_se['close'].view()

    second_ne = ctrl.Antecedent(np.arange(0, 4, 0.01), 'second_ne')
    second_ne['close'] = fuzz.trapmf(second_ne.universe, [0, 0, 0.7, 0.8])
    second_ne['far'] = fuzz.trapmf(second_ne.universe, [0.7, 0.8, 4, 4])
    # second_ne['close'].view()

    second_sw = ctrl.Antecedent(np.arange(0, 4, 0.01), 'second_sw')
    second_sw['close'] = fuzz.trapmf(second_sw.universe, [0, 0, 0.9, 1.3])
    second_sw['far'] = fuzz.trapmf(second_sw.universe, [0.9, 1.3, 4, 4])
    # second_sw['close'].view()

    right_speed = ctrl.Consequent(np.arange(-4, 4, 0.01), 'right_speed')
    right_speed['very slow'] = fuzz.trapmf(right_speed.universe, [-4, -4, -2, -1])
    right_speed['slow'] = fuzz.trimf(right_speed.universe, [-2, -1, -0])
    right_speed['medium'] = fuzz.trimf(right_speed.universe, [-1, 0, 1])
    right_speed['high'] = fuzz.trimf(right_speed.universe, [0, 1, 2])
    # right_speed['slow'].view()

    left_vel = ctrl.Consequent(np.arange(-4, 4, 0.01), 'left_speed')
    left_vel['very slow'] = fuzz.trapmf(left_vel.universe, [-4, -4, -4, -2])
    left_vel['slow'] = fuzz.trimf(left_vel.universe, [-4, -2, 0])
    left_vel['medium'] = fuzz.trimf(left_vel.universe, [-2, 0, 2])
    left_vel['high'] = fuzz.trimf(left_vel.universe, [0, 2, 4])
    # left_vel['slow'].view()

    searching_speed = ctrl.Consequent(np.arange(-10, 10, 0.01), 'searching_speed')
    searching_speed['slow'] = fuzz.trimf(searching_speed.universe, [-3, 0, 3])
    searching_speed['high'] = fuzz.trapmf(searching_speed.universe, [2, 6, 10, 10])
    # searching_speed['slow'].view()


def build_rules():
    global backward, forward, searching_speed

    back_rule1 = ctrl.Rule(sw['medium'], right_speed['very slow'])
    back_rule2 = ctrl.Rule(sw['close'], right_speed['medium'])
    back_rule3 = ctrl.Rule(sw['far'], left_vel['very slow'])
    back_rule4 = ctrl.Rule(ne['medium'], left_vel['medium'])
    back_rule5 = ctrl.Rule(ne['slow'], left_vel['slow'])
    back_rule6 = ctrl.Rule(sw['far'], right_speed['medium'])
    back_rule7 = ctrl.Rule(sw['medium'], left_vel['very slow'])
    back_rule8 = ctrl.Rule(sw['close'], left_vel['medium'])

    backward_ctrl = ctrl.ControlSystem(
        [back_rule1, back_rule2, back_rule3, back_rule4, back_rule5, back_rule6, back_rule7, back_rule8])
    backward = ctrl.ControlSystemSimulation(backward_ctrl)

    forward_rule1 = ctrl.Rule(second_sw['close'] & second_ne['far'], right_speed['high'])
    forward_rule2 = ctrl.Rule(second_sw['close'] & second_ne['far'], left_vel['high'])
    forward_rule3 = ctrl.Rule(second_ne['close'] & second_sw['far'], right_speed['medium'])
    forward_rule4 = ctrl.Rule(second_ne['close'] & second_sw['far'], left_vel['medium'])
    forward_rule5 = ctrl.Rule(second_sw['far'], right_speed['medium'])
    forward_rule6 = ctrl.Rule(second_sw['far'], left_vel['medium'])

    forward_ctrl = ctrl.ControlSystem(
        [forward_rule1, forward_rule2, forward_rule3, forward_rule4, forward_rule5, forward_rule6])
    forward = ctrl.ControlSystemSimulation(forward_ctrl)


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
    global t, tank, stop, is_straightening, is_place_found
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

    tank.forward(7)
    time.sleep(10)
    t = time.time()
    stop = False  # used to stop the tank at the beginning
    is_straightening = False  # used after driving back to park the tank in line with other cars
    is_place_found = False


def run_simulation():
    global se_dist, ne_dist, stop, is_straightening, is_place_found

    tank.rightvelocity = 3  # searching_speed.output['searching_speed']
    tank.leftvelocity = 3  # searching_speed.output['searching_speed']
    tank.setVelocity()

    while (time.time() - t) < 300:
        ne_dist = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 83, vrep.simx_opmode_buffer)[2])
        se_dist = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 78, vrep.simx_opmode_buffer)[2])
        sw_dist = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 79, vrep.simx_opmode_buffer)[2])

        if not is_place_found:
            if se_dist >= 2.8 and ne_dist < 1.31:
                print('Going for the spot!')
                is_place_found = True

        if is_place_found:
            forward.input['second_sw'] = sw_dist
            forward.input['second_ne'] = ne_dist
            forward.compute()

            backward.input['sw'] = sw_dist
            backward.input['ne'] = ne_dist
            backward.compute()

            if is_straightening:
                right_speed = forward.output['right_speed']
                left_speed = forward.output['left_speed']
            else:
                right_speed = backward.output['right_speed']
                left_speed = backward.output['left_speed']

            tank.rightvelocity = right_speed
            tank.leftvelocity = left_speed
            tank.setVelocity()

            if sw_dist < 0.91 and forward.output['left_speed'] > 0.1 and not is_straightening:
                print('Straightening')
                is_straightening = True

        time.sleep(0.2)


def run_all():
    build_controls()
    build_rules()
    connect_to_vrep()
    setup_proximity_sensors()
    run_simulation()


run_all()
