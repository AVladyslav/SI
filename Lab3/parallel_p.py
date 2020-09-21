import vrep
# import sim as vrep
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

# create instance of Tank
tank = Tank(clientID)


def set_sensors():
    global proximity_sensors, proximity_sensors_handles, t

    proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
    proximity_sensors_handles = [0] * 8

    # get handle to proximity sensors
    for i in range(len(proximity_sensors)):
        _, proximity_sensors_handles[i] = vrep.simxGetObjectHandle(clientID,
                                                                   "Proximity_sensor_" + proximity_sensors[i],
                                                                   vrep.simx_opmode_blocking)
    # read and print values from proximity sensors
    # first reading should be done with simx_opmode_streaming, further with simx_opmode_buffer parameter
    for sensor_name, sensor_handle in zip(proximity_sensors, proximity_sensors_handles):
        vrep.simxReadProximitySensor(clientID, sensor_handle, vrep.simx_opmode_streaming)

    tank.forward(7)
    time.sleep(10)
    t = time.time()


def create_mf():
    global se, ne, en, sw, ne_2, nw, preparation_velocity, right_velocity, left_velocity

    se = ctrl.Antecedent(np.arange(0, 4, 0.01), 'se')
    se['close'] = fuzz.trapmf(se.universe, [0, 0, 1.5, 2.8])
    se['far'] = fuzz.trapmf(se.universe, [1.5, 2.8, 4, 4])

    ne = ctrl.Antecedent(np.arange(0, 4, 0.01), 'ne')
    ne['close'] = fuzz.trapmf(ne.universe, [0, 0, 0.5, 2])
    ne['far'] = fuzz.trapmf(ne.universe, [0.5, 2, 4, 4])

    preparation_velocity = ctrl.Consequent(np.arange(-4, 10, 0.01), 'preparation_velocity')
    preparation_velocity['low'] = fuzz.trimf(preparation_velocity.universe, [-4, 0, 4])
    preparation_velocity['high'] = fuzz.trapmf(preparation_velocity.universe, [3, 6, 10, 10])

    en = ctrl.Antecedent(np.arange(0, 3, 0.01), 'en')
    en['close'] = fuzz.trapmf(en.universe, [0, 0, 1.2, 1.5])
    en['far'] = fuzz.trapmf(en.universe, [1.2, 1.5, 3, 3])

    sw = ctrl.Antecedent(np.arange(0, 4, 0.01), 'sw')
    sw['close'] = fuzz.trapmf(se.universe, [0, 0, 1.15, 2])
    sw['far'] = fuzz.trapmf(se.universe, [1.15, 2, 4, 4])

    ne_2 = ctrl.Antecedent(np.arange(0, 4, 0.01), 'ne_2')
    ne_2['close'] = fuzz.trapmf(ne.universe, [0, 0, 1.15, 2])
    ne_2['far'] = fuzz.trapmf(ne.universe, [1.15, 2, 4, 4])

    nw = ctrl.Antecedent(np.arange(0, 10, 0.01), 'nw')
    nw['close'] = fuzz.trapmf(nw.universe, [0, 0, 1.15, 2])
    nw['far'] = fuzz.trapmf(nw.universe, [1.15, 2, 4, 10])

    right_velocity = ctrl.Consequent(np.arange(-5, 5, 0.01), 'right_velocity')
    right_velocity['negative_small'] = fuzz.trapmf(right_velocity.universe, [-4, -2.5, -1.5, -0.5])
    right_velocity['zero'] = fuzz.trapmf(right_velocity.universe, [-1.5, -0.5, 0.5, 1.5])
    right_velocity['positive_small'] = fuzz.trapmf(right_velocity.universe, [0.5, 1.5, 2.5, 4])

    left_velocity = ctrl.Consequent(np.arange(-5, 5, 0.01), 'left_velocity')
    left_velocity['negative_small'] = fuzz.trapmf(left_velocity.universe, [-4, -2.5, -1.5, -0.5])
    left_velocity['zero'] = fuzz.trapmf(left_velocity.universe, [-1.5, -0.5, 0.5, 1.5])
    left_velocity['positive_small'] = fuzz.trapmf(left_velocity.universe, [0.5, 1.5, 2.5, 4])


def create_rules():
    global preparation_velocity, backward_velocity, first_s_velocity, second_s_velocity

    prep_rule1 = ctrl.Rule(se['close'] & ne['far'], preparation_velocity['high'])
    prep_rule2 = ctrl.Rule(se['far'] & ne['close'], preparation_velocity['low'])
    prep_ctrl = ctrl.ControlSystem([prep_rule1, prep_rule2])
    preparation_velocity = ctrl.ControlSystemSimulation(prep_ctrl)

    first_s_rule1 = ctrl.Rule(en['close'], right_velocity['positive_small'])
    first_s_rule2 = ctrl.Rule(en['far'] & se['close'], right_velocity['zero'])
    first_s_rule3 = ctrl.Rule(en['close'], left_velocity['negative_small'])
    first_s_rule4 = ctrl.Rule(en['far'] & se['close'], left_velocity['zero'])
    first_s_ctrl = ctrl.ControlSystem([first_s_rule1, first_s_rule2, first_s_rule3, first_s_rule4])
    first_s_velocity = ctrl.ControlSystemSimulation(first_s_ctrl)

    backward_rule1 = ctrl.Rule(sw['far'], right_velocity['negative_small'])
    backward_rule2 = ctrl.Rule(sw['far'], left_velocity['negative_small'])
    backward_rule3 = ctrl.Rule(sw['close'] & ne_2['close'], right_velocity['zero'])
    backward_rule4 = ctrl.Rule(sw['close'] & ne_2['close'], left_velocity['zero'])
    backward_ctrl = ctrl.ControlSystem([backward_rule1, backward_rule2, backward_rule3, backward_rule4])
    backward_velocity = ctrl.ControlSystemSimulation(backward_ctrl)

    second_s_rule1 = ctrl.Rule(nw['far'], right_velocity['negative_small'])
    second_s_rule2 = ctrl.Rule(nw['far'], left_velocity['positive_small'])
    second_s_rule3 = ctrl.Rule(nw['close'], right_velocity['zero'])
    second_s_rule4 = ctrl.Rule(nw['close'], left_velocity['zero'])
    second_s_ctrl = ctrl.ControlSystem([second_s_rule1, second_s_rule2, second_s_rule3, second_s_rule4])
    second_s_velocity = ctrl.ControlSystemSimulation(second_s_ctrl)


set_sensors()
create_mf()
create_rules()

is_preparation = True
is_first_s = False
is_backward = False
is_second_s = False

tank.forward(5)

# continue reading and printing values from proximity sensors
t = time.time()
while (time.time() - t) < 200:  # read values for 200 seconds

    # Handles no: NE - 83, SE - 78, EN - 76, SW - 79, NW - 82
    ne_distance = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 83, vrep.simx_opmode_buffer)[2])
    se_distance = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 78, vrep.simx_opmode_buffer)[2])
    en_distance = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 76, vrep.simx_opmode_buffer)[2])
    sw_distance = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 79, vrep.simx_opmode_buffer)[2])
    nw_distance = np.linalg.norm(vrep.simxReadProximitySensor(clientID, 82, vrep.simx_opmode_buffer)[2])

    if is_preparation:

        preparation_velocity.input['se'] = se_distance
        preparation_velocity.input['ne'] = ne_distance
        preparation_velocity.compute()
        velocity = preparation_velocity.output['preparation_velocity']
        tank.forward(velocity)
        if (velocity == 0.0) or (se_distance > 2.7 and 0 < ne_distance < 1.5):
            is_preparation = False
            is_first_s = True
            print('First half S')

    elif is_first_s:

        first_s_velocity.input['se'] = se_distance
        first_s_velocity.input['en'] = en_distance
        first_s_velocity.compute()
        right_velocity = first_s_velocity.output['right_velocity']
        left_velocity = first_s_velocity.output['left_velocity']
        tank.rightvelocity = right_velocity
        tank.leftvelocity = left_velocity
        tank.setVelocity()
        if round(left_velocity, 2) == 0.0 or round(right_velocity, 2) == 0.0:
            is_first_s = False
            is_backward = True
            print('Backward')

    elif is_backward:

        backward_velocity.input['sw'] = sw_distance
        backward_velocity.input['ne_2'] = ne_distance
        backward_velocity.compute()
        right_velocity = backward_velocity.output['right_velocity']
        left_velocity = backward_velocity.output['left_velocity']
        tank.rightvelocity = right_velocity
        tank.leftvelocity = left_velocity
        tank.setVelocity()

        if sw_distance < 1.2 and ne_distance > 1:

            is_backward = False
            is_second_s = True
            print('Second half S')

    elif is_second_s:

        second_s_velocity.input['nw'] = nw_distance
        second_s_velocity.compute()
        right_velocity = second_s_velocity.output['right_velocity']
        left_velocity = second_s_velocity.output['left_velocity']
        tank.rightvelocity = right_velocity
        tank.leftvelocity = left_velocity
        tank.setVelocity()
