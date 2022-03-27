import time
from math import atan2, degrees, cos, isclose
import RPi.GPIO
import board
import digitalio
import pwmio
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import adafruit_hcsr04
import adafruit_lsm303dlh_mag
import numpy as np
import skfuzzy as fuzz
from node import Node, Connection
#import matplotlib.pyplot as plt
#ensure gpios are clean
RPi.GPIO.cleanup()
#create objects for each sesnor, f/b = front/back l/m/r = left/middle/right
sonarfl = adafruit_hcsr04.HCSR04(trigger_pin=board.D9, echo_pin=board.D11)  # 9, 11
sonarfm = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)
sonarfr = adafruit_hcsr04.HCSR04(trigger_pin=board.D22, echo_pin=board.D10)  # 22, 10
sonarbr = adafruit_hcsr04.HCSR04(trigger_pin=board.D17, echo_pin=board.D27)  # 17,27
sonarbm = adafruit_hcsr04.HCSR04(trigger_pin=board.D14, echo_pin=board.D15)  # 24,25
sonarbl = adafruit_hcsr04.HCSR04(trigger_pin=board.D25, echo_pin=board.D18)  # 18, 23
fl = 0
fm = 0
fr = 0
br = 0
bm = 0
bl = 0

i2c = board.I2C()  # uses board.SCL and board.SDA initates i2c communcation for lsm303dlhc

# set target vector and start node
target = Node(vector=np.array([15,10])) # should change this to be an input. Provides a vector of x,y distances in cm for target from current position
current_node = Node()
current_node.evaluate(target)
node_list = []
node_list.append(current_node)
explored_list = []

sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
fr1 = digitalio.DigitalInOut(board.D20)  # front right motor pair
fr1.direction = digitalio.Direction.OUTPUT
fr2 = digitalio.DigitalInOut(board.D21)
fr2.direction = digitalio.Direction.OUTPUT

br1 = digitalio.DigitalInOut(board.D16)  # 19 back right motor pair
br1.direction = digitalio.Direction.OUTPUT
br2 = digitalio.DigitalInOut(board.D12)  # 26
br2.direction = digitalio.Direction.OUTPUT

fl1 = digitalio.DigitalInOut(board.D7)  # front left motor pair
fl1.direction = digitalio.Direction.OUTPUT
fl2 = digitalio.DigitalInOut(board.D8)
fl2.direction = digitalio.Direction.OUTPUT

bl1 = digitalio.DigitalInOut(board.D19)  # 12 back left motor pair
bl1.direction = digitalio.Direction.OUTPUT
bl2 = digitalio.DigitalInOut(board.D26)  # 16
bl2.direction = digitalio.Direction.OUTPUT

def destroy():
    RPi.GPIO.cleanup()
    print("\nCleaned up GPIO resources.")


def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    return vector_2_degrees(magnet_x, magnet_y)


def setup():
    roomdeg = get_heading(sensor)
    roomdeg = -1*roomdeg
    print("set up")
    return roomdeg


def headchange(goalhead, change):
    goalhead = goalhead + change
    if 360 <= goalhead:
        goalhead = goalhead - 360
    if goalhead < 0:
        goalhead = goalhead + 360
    return goalhead

def motors(leftforward,leftback, rightcycle,rightback):
    fr1.duty_cycle = rightcycle*65535  # right motors
    fr2.duty_cycle = rightback*65535
    br1.duty_cycle = rightcycle*65535
    br2.duty_cycle = rightback*65535

    fl1.duty_cycle = leftforward*65535  # left motors
    fl2.duty_cycle = leftback*65535
    bl1.duty_cycle = leftforward*65535
    bl2.duty_cycle = leftback*65535


def evaluate_path():
    path = []
    path.append(current_node)
    global node_list
    global explored_list
    global target
    no_path = True
    searches = 0
    
    while (no_path):
        searches += 1

        if (check_los(current_node, target)):
            move_to_node(target)
            no_path = False
            break

        # check for best connected unexplored node
        if (not current_node.isExplored()):
            explore_node(current_node)
            current_node.setExplored()
            explored_list.append(current_node)
        
        best_connected = search_node(current_node)

        # search for best explored node
        best_node = current_node
        best_score = best_node.get_score()
        for node in explored_list:
            if (node.get_score() < best_score):
                best_score = node.get_score()
                best_node = node

        if (best_connected.get_score() >  best_score):
            move_to_node(best_connected)
        else:
            path_to_node(best_node)

def path_to_node(target_node):
    global current_node
    path = search_for_path(current_node, target_node)
    for node in path:
        if (check_los(current_node, target_node)):
            move_to_node(target_node)
            current_node = target_node
            break
        else:
            move_to_node(node)
            current_node = node

def search_for_path(start_node, target_node):
    path_from_start = []
    path_from_target = []

    # finds path from target node back to origin

    previous = target_node.get_previous()
    while (not previous.is_target(node_list[0].get_vector())):
        path_from_target.append(previous)
        previous = previous.get_previous()

    # compares previous nodes to find last common node, then derives path

    previous = start_node.get_previous()
    while (not previous.is_target(node_list[0].get_vector())):
        
        for node in path_from_target:
            if (previous.is_target(node.get_vector())):
                index = path_from_target.index(node)
                path = path_from_start + path_from_target[0::index]
                return path
            else:
                path_from_start.append(previous)
                previous = previous.get_previous()        

def search_node(node):
    connections = node.get_connections()
    # low scores are good, as it indicates lower distance
    best_score = node.get_score()
    best_node = node
    for connection in connections():
        if (not connection.get_node().isExplored()):
            if (connection.get_node().get_score() < best_score):
                best_score = connection.get_node().get_score()
                best_node = connection.get_node()
    return best_node

def check_los(start_node, target_node):
    #check LoS between current node and another node
    global sonarfm
    move_vector = target_node.get_vector() - start_node.get_vector()
    turn_to(vector_to_bearing(move_vector))
    if (sonarfm.distance >= vector_to_distance(move_vector)):
        start_node.add_connection(target_node, move_vector)
        target_node.add_connection(start_node, -move_vector)
        return True
    else:
        return False

def explore_node(node):
    # add movement routine to spin on the spot,
    # stopping in increments to scan
    # could at later date use a more focused exploration
    # range to optimise time and space complexity
    global sonarfm

    for i in range(12):
        bearing =  30 * i
        turn_to(bearing)
        clearance = sonarfm.distance

        # create nodes along every 10cm of clearance

        distance = 10
        
        while (distance < clearance):
            # generate a vector
            vector = bearing_to_vector(distance, bearing)
            coord = node.get_vector() + vector

            # generate node and connections
            new_node = Node(vector=coord)
            new = True
            for n in node_list:
                if (new_node.is_target(n)):
                    node.add_connection(n, vector)
                    n.add_connection(node, -vector)
                    new = False
                    break
            if (new):
                node.add_connection(new_node, vector)
                new_node.add_connection(node, -vector)
                new_node.evaluate(target)
                node_list.append(new_node)
            
            distance += 10
            

def move_to_node(target_node):
    move_vector = target_node.get_vector() - current_node.get_vector()
    turn_to(vector_to_bearing(move_vector))
    move_to(vector_to_distance(move_vector))

    print("moving along vector " + move_vector + " to reach " + target_node.get_vector())
    
def bearing_to_vector(distance, bearing):
    x = distance * np.sin(bearing)
    y = distance * np.cos(bearing)
    return np.array([x, y])

def vector_to_bearing(vector):
    x = vector[0]
    y = vector[1]
    return np.arctan((x/y))

def vector_to_distance(vector):
    x = vector[0]
    y = vector[1]
    return np.sqrt(np.square(x) + np.square(y))

def move_to(distance):
    global sonarfm
    current_distance = sonarfm.distance
    target_distance = current_distance - distance

    if (target_distance < 0):
        backward()
    else:
        forward()
    
    while (not isclose(current_distance, target_distance, abs_tol=1)):
        current_distance = sonarfm.distance
    else:
        stop()

def turn_to(bearing):
    global sensor
    current_bearing = get_heading(sensor)

    if (bearing > current_bearing):
        right()
    else:
        left()
    
    while (not isclose(current_bearing, bearing, abs_tol=5)):
        current_bearing = get_heading(sensor)
    else:
        stop()
    


def forward():
    print("forward")
    global fr1, fr2, br1, br2, fl1, fl2, bl1, bl2
    fr1.value = 1
    fr2.value = 0
    br1.value = 1
    br2.value = 0

    fl1.value = 1
    fl2.value = 0
    bl1.value = 1
    bl2.value = 0

def backward():
    print("backward")
    global fr1, fr2, br1, br2, fl1, fl2, bl1, bl2
    fr1.value = 0
    fr2.value = 1
    br1.value = 0
    br2.value = 1

    fl1.value = 0
    fl2.value = 1
    bl1.value = 0
    bl2.value = 1

def left():
    print("left")
    global fr1, fr2, br1, br2, fl1, fl2, bl1, bl2
    fr1.value = 1
    fr2.value = 0
    br1.value = 1
    br2.value = 0

    fl1.value = 0
    fl2.value = 1
    bl1.value = 0
    bl2.value = 1

def right():
    print("right")
    global fr1, fr2, br1, br2, fl1, fl2, bl1, bl2
    fr1.value = 0
    fr2.value = 1
    br1.value = 0
    br2.value = 1

    fl1.value = 1
    fl2.value = 0
    bl1.value = 1
    bl2.value = 0

def stop():
    print("stop")
    global fr1, fr2, br1, br2, fl1, fl2, bl1, bl2
    fr1.value = 0
    fr2.value = 0
    br1.value = 0
    br2.value = 0

    fl1.value = 0
    fl2.value = 0
    bl1.value = 0
    bl2.value = 0



if __name__ == '__main__':
    #print("go!")
    #setup()
    #print("setup")
    try:
        #print("try loop!")
        evaluate_path()
        #print("exit loop?")
    except KeyboardInterrupt:
       # print("destroy")
        destroy()
        print("destroyed!")
