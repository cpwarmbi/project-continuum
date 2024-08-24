"""
\file       pi_main.py
\brief      RPi Code for interfacing with Lidar sensor
            Communicates with Pico via UART to send motor control signals
            Posts to server on remote laptop for data visualization

\authors    Corbin Warmbier | corbinwarmbier@gmail.com

\date       Initial: 07/13/24  |  Last: 07/13/24
"""
""" [Imports] """
import math
import numpy as np
from adafruit_rplidar import RPLidar, RPLidarException
import requests
import json
import serial
import RPi.GPIO as GPIO
import time

""" [Constants] """
UART_RDY_PIN = 23
UART_PI_2_PICO_PIN = 24
PICO_DISABLE_PIN = 25
PICO_RDY_PIN = 16
Ksd = 0.15
Kfw = 0.008
Ki = 0.000015

""" [Initializations] """
ser = serial.Serial(
    port = '/dev/ttyS0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    bytesize=serial.EIGHTBITS,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

# Setup UART_Rdy pin on RPi (Dummy Interrupt)
UART_Rdy = 0  # Global flag for UART reading
GPIO.setmode(GPIO.BCM)
GPIO.setup(UART_RDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(UART_PI_2_PICO_PIN, GPIO.OUT)
GPIO.setup(PICO_DISABLE_PIN, GPIO.OUT)
GPIO.setup(PICO_RDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

LIDAR_PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, LIDAR_PORT_NAME, timeout = 3)

travel_distance = 0
max_distance = 0
scan_data = [0]*360

tmp_cnt = 0
max_kick = 0.1

counter = 0
kick_cd_counter = 0

fw_integral = 0
pico_rdy = 0

""" [Local Functions] """
def uart_irq_handler(channel):
    """
    Handles interrupts received on UART_RDY_PIN
    Sets UART_Rdy global flag when data has been received

    return: none
    """
    global UART_Rdy
    global travel_distance
    # Sanity Check If Statement
    if channel == UART_RDY_PIN:
        travel_distance = float(ser.readline().decode())
        #print(f"Travel Distance {travel_distance}")
        UART_Rdy = 1
        if travel_distance is None:
            UART_Rdy = -1

def pico_rdy_irq_handler(channel):
    global pico_rdy
    if channel == PICO_RDY_PIN:
        pico_rdy = 1

def write_pid_ctrl(direction, percent_ang, brake):
    #Write to Pico
    GPIO.output(UART_PI_2_PICO_PIN, True)
    message = direction + ' ' + str(percent_ang) + ' ' + brake + '\n'
    #print(message)
    ser.write(message.encode())
    ser.flush()
    GPIO.output(UART_PI_2_PICO_PIN, False)


def find_smallest_angle(vectors):
    min_distance = vectors[0][1]
    for angle, distance in vectors:
        if(distance < min_distance):
            min_distance = distance
            smallest_angle = angle
    return smallest_angle, min_distance


def find_longest_string_of_zeros(arr):
    max_length = 0
    current_length = 0
    max_start_index = -1
    max_end_index = -1
    current_start_index = -1

    for i, num in (arr):
        if num == 0:
            if current_length == 0:
                current_start_index = i  # Start of a new sequence of 0s
            current_length += 1
        else:
            if current_length > max_length:
                max_length = current_length
                max_start_index = current_start_index
                max_end_index = i - 1
            current_length = 0

    # Final check in case the array ends with a sequence of 0s
    if current_length > max_length:
        max_length = current_length
        max_start_index = current_start_index
        max_end_index = len(arr) - 1

    return max_length, max_start_index, max_end_index


def PID_control(scan_data):
    """
    Handles PID control calculations based on input scan_data
    
    return: (motor_spd: int, motor_dir: char, servo_ang: int, servo_dir: char)
      * motor_spd and servo_ang are returned as a percentage of the maximum i.e. (0 - 100%)
    """
    global fw_integral
    global pico_rdy
    rh_vectors = []
    lh_vectors = []
    fw_vectors = []
    for angle, distance in enumerate(scan_data):
        # Process Right Hand Vectors
        if (angle >= 0 and angle <= 20) or (angle <= 360 and angle >= 340):
            lh_vectors.append((angle, distance))
        elif (angle >= 160 and angle <= 200):
            rh_vectors.append((angle, distance))
        if (angle >= 0 and angle <= 180):
            fw_vectors.append((angle, distance))
            # print(fw_vectors)
        else:
            pass  # for now

    """ Target Angle """
    # Get the longest string of zeros array (the angle we want to be)
    zero_length, zero_start, zero_end = find_longest_string_of_zeros(fw_vectors)
    largest_sum_index = -1
    max_sum = 0
    brake = 'F'
    target_distance = 0
    if zero_length < 5:
        print("!!Non Zero Target Selected!!")
        for i in range(len(fw_vectors) -5):
            current_sum = sum(fw_vectors[j][1] for j in range(i, i + 5))
            if current_sum > max_sum:
                max_sum = current_sum
                largest_sum_index = i
                target_distance = max_sum / 5
        if target_distance > 1250:
            target_angle = fw_vectors[largest_sum_index + 2][0]
        else:
            brake = 'N'
    else:
        target_angle = int((zero_start + zero_end) / 2)

    percent_ang = 0
    fw_error = (target_angle - 90)
    fw_deadband = 2
    
    """ Proportion """
    # Check if fw error is greater than deadband and set percent ang if so
    if fw_error > 90 + fw_deadband or fw_error < 90 - fw_deadband:
        percent_ang = abs(fw_error) * Kfw
        percent_ang = min(0.6, percent_ang)
        if target_angle < 90:
            percent_ang = -percent_ang
        fw_ang = percent_ang

    """ Integral """
    # If Pico Is Ready, Enable Integral Term
    if pico_rdy == 1:
        if target_angle < 90:
            fw_integral += fw_error * Ki
            fw_integral = max(-0.3, fw_integral)
        else:
            fw_integral += fw_error * Ki
            fw_integral = min(0.3, fw_integral)
    percent_ang += fw_integral

    """ Obstacle Avoidance """
    obstacle_avoid_tol = 500
    obstacle_ang = 0
    angle_left = -1
    angle_right = -1
    dist_short_l = obstacle_avoid_tol + 1
    dist_short_r = obstacle_avoid_tol + 1
    for (angle, distance) in fw_vectors[zero_start-5: zero_start]:
        if distance <= obstacle_avoid_tol and distance < dist_short_l and distance > 0:
            angle_left = angle
            dist_short_l = distance
    if(dist_short_l < obstacle_avoid_tol):
        print(f"Close Wall Detected Left at Ang: {angle_left} Dist: {dist_short_l}")
    
    for (angle, distance) in fw_vectors[zero_end: zero_end + 5]:
        if distance <= obstacle_avoid_tol and distance < dist_short_r and distance > 0:
            angle_right = angle
            dist_short_r = distance

    if(dist_short_r < obstacle_avoid_tol):
        print(f"Close Wall Detected Right at Ang: {angle_right} Dist: {dist_short_r}")
    
    # Krabby Patty Secret Formula
    if(dist_short_r < dist_short_l):
        obstacle_ang = 0.2 + (-0.05 / 350) * (dist_short_r - 150)
        percent_ang += obstacle_ang
    elif(dist_short_r > dist_short_l):
        obstacle_ang = 0.2 + (-0.05 / 350) * (dist_short_l - 150) 
        percent_ang -= obstacle_ang
    
    print(f"T Ang: {target_angle} T Dist: {target_distance}\n+ Fw P%: {round(fw_ang, 3)}  Fw I%: {round(fw_integral, 3)}  Ob %: {round(obstacle_ang, 3)}")
    
    # Determine Direction to Turn
    if percent_ang > 0:
        direction = 'R'
    elif percent_ang < 0:
        direction = 'L'
    else:
        direction = 'N'
    
    print(f"=== [Total % {round(percent_ang, 3)} Dir {direction} Motor Dir {brake}] ===\n\n")
    return direction, abs(percent_ang), brake


def process_data(data):
    global tmp_cnt
    global UART_Rdy
    global travel_distance
    if UART_Rdy != 1:
        travel_distance = float(UART_Rdy)
    data_json = {
        "scan_data" : data,
        "distance" : travel_distance
    }
    # For Mapping do rh_vector and lh_vector but with 360 measurements (and 0's in the non ideals)
    UART_Rdy = 0
    travel_distance = 0
    direction, percent_ang ,brake = PID_control(data)
    write_pid_ctrl(direction, percent_ang, brake)
    print(json.dumps(data_json))
    requests.post('http://10.42.0.61:8069', json=data_json)

if True:
    # Setup Interrupt Handler (what is bouncetime?)
    GPIO.output(UART_PI_2_PICO_PIN, False)
    GPIO.output(PICO_DISABLE_PIN, False)
    GPIO.add_event_detect(UART_RDY_PIN, GPIO.RISING, callback=uart_irq_handler, bouncetime=200)
    GPIO.add_event_detect(PICO_RDY_PIN, GPIO.RISING, callback=pico_rdy_irq_handler, bouncetime=200)
    try:
        print("=== [Beginning Lidar Scans] ===")
        for scan in lidar.iter_scans():
            for(quality, angle, distance) in scan:
                scan_data[min([359, math.floor(angle)])] = distance
            process_data(scan_data)
            scan_data = [0]*360
    except RPLidarException as e:
        print("Error has occured with LiDar. Shutting Down ")
        print(e)
        GPIO.output(PICO_DISABLE_PIN, True)
        lidar.stop()
        lidar.stop_motor()
        lidar.clear_input()
        lidar.disconnect()
        GPIO.output(PICO_DISABLE_PIN, False)
    except KeyboardInterrupt:
        print("Stopping")   
        GPIO.output(PICO_DISABLE_PIN, True)
        lidar.stop()
        lidar.stop_motor()
        lidar.clear_input()
        lidar.disconnect()
        GPIO.output(PICO_DISABLE_PIN, False)
