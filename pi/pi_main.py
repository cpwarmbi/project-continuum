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
from scipy import interpolate
import requests
import json
import serial
import RPi.GPIO as GPIO
import time
import berryIMU
import sys
import datetime
import os
import IMU

""" [Constants] """
UART_RDY_PIN = 23
UART_PI_2_PICO_PIN = 24
PICO_DISABLE_PIN = 25
PICO_RDY_PIN = 16
Ksd = 0.15
Kfw = 0.008
Ki = 0.000015
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070                  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40                      # Complementary filter constant
MAG_LPF_FACTOR = 0.4            # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4            # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay

""" [Compass Calibration Offsets] """
magXmin =  -2019
magYmin =  -764
magZmin =  -2038
magXmax =  -14
magYmax =  1275
magZmax =  -1555

""" [Initializations] """
ser = serial.Serial(
    port = '/dev/ttyS0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    bytesize=serial.EIGHTBITS,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
LIDAR_PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, LIDAR_PORT_NAME, timeout = 3)

""" [Pin Init] """
GPIO.setmode(GPIO.BCM)
GPIO.setup(UART_RDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(UART_PI_2_PICO_PIN, GPIO.OUT)
GPIO.setup(PICO_DISABLE_PIN, GPIO.OUT)
GPIO.setup(PICO_RDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

""" [Globals] """
UART_Rdy = 0  # Global flag for UART reading
pico_rdy = 0
travel_distance = 0
max_distance = 0
scan_data = [0]*360
tmp_cnt = 0
max_kick = 0.1
counter = 0
kick_cd_counter = 0
fw_integral = 0

# Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

# Sensor reading variables
gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
CFangleXFiltered = 0.0
CFangleYFiltered = 0.0
kalmanX = 0.0
kalmanY = 0.0
oldXMagRawValue = 0
oldYMagRawValue = 0
oldZMagRawValue = 0
oldXAccRawValue = 0
oldYAccRawValue = 0
oldZAccRawValue = 0
prev_time = datetime.datetime.now()

#Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

""" [Interrupt Handlers] """
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

""" [Local Functions] """
def kalmanFilterY (accAngle, gyroRate, DT):
    y=0.0
    S=0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
    YP_01 = YP_01 + ( - DT * YP_11 )
    YP_10 = YP_10 + ( - DT * YP_11 )
    YP_11 = YP_11 + ( + Q_gyro * DT )

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + ( K_0 * y )
    y_bias = y_bias + ( K_1 * y )

    YP_00 = YP_00 - ( K_0 * YP_00 )
    YP_01 = YP_01 - ( K_0 * YP_01 )
    YP_10 = YP_10 - ( K_1 * YP_00 )
    YP_11 = YP_11 - ( K_1 * YP_01 )

    return KFangleY


def kalmanFilterX (accAngle, gyroRate, DT):
    x=0.0
    S=0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11


    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
    XP_01 = XP_01 + ( - DT * XP_11 )
    XP_10 = XP_10 + ( - DT * XP_11 )
    XP_11 = XP_11 + ( + Q_gyro * DT )

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + ( K_0 * x )
    x_bias = x_bias + ( K_1 * x )

    XP_00 = XP_00 - ( K_0 * XP_00 )
    XP_01 = XP_01 - ( K_0 * XP_01 )
    XP_10 = XP_10 - ( K_1 * XP_00 )
    XP_11 = XP_11 - ( K_1 * XP_01 )

    return KFangleX


def compass_read():
    global gyroXangle
    global gyroYangle
    global gyroZangle
    global CFangleX
    global CFangleY
    global CFangleXFiltered
    global CFangleYFiltered
    global kalmanX
    global kalmanY
    global oldXMagRawValue
    global oldYMagRawValue
    global oldZMagRawValue
    global oldXAccRawValue
    global oldYAccRawValue
    global oldZAccRawValue
    global prev_time
    global acc_medianTable1X
    global acc_medianTable1Y
    global acc_medianTable1Z
    global acc_medianTable2X
    global acc_medianTable2Y
    global acc_medianTable2Z
    global mag_medianTable1X
    global mag_medianTable1Y
    global mag_medianTable1Z
    global mag_medianTable2X
    global mag_medianTable2Y
    global mag_medianTable2Z

    # Read the accelerometer, gyroscope and magnetometer values
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    # Apply compass calibration
    MAGx -= (magXmin + magXmax) /2
    MAGy -= (magYmin + magYmax) /2
    MAGz -= (magZmin + magZmax) /2

    # Calculate loop Period(LP). How long between Gyro Reads
    b = datetime.datetime.now() - prev_time
    prev_time = datetime.datetime.now()
    LP = b.microseconds/(1000000*1.0)

    """======================================"""
    """          [Low Pass Filter]           """
    """======================================"""

    MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR)
    MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR)
    MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR)
    ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR)
    ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR)
    ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR)

    oldXMagRawValue = MAGx
    oldYMagRawValue = MAGy
    oldZMagRawValue = MAGz
    oldXAccRawValue = ACCx
    oldYAccRawValue = ACCy
    oldZAccRawValue = ACCz

    """======================================"""
    """        [End Low Pass Filter]         """
    """======================================"""


    """======================================"""
    """   [Median Filter for Acceleromter]   """
    """======================================"""

    # Cycle the table
    for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
        acc_medianTable1X[x] = acc_medianTable1X[x-1]
        acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
        acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

    # Insert the lates values
    acc_medianTable1X[0] = ACCx
    acc_medianTable1Y[0] = ACCy
    acc_medianTable1Z[0] = ACCz

    # Copy the tables
    acc_medianTable2X = acc_medianTable1X[:]
    acc_medianTable2Y = acc_medianTable1Y[:]
    acc_medianTable2Z = acc_medianTable1Z[:]

    # Sort table 2
    acc_medianTable2X.sort()
    acc_medianTable2Y.sort()
    acc_medianTable2Z.sort()

    # The middle value is the value we are interested in
    ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)]
    ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)]
    ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)]

    """======================================"""
    """ [End Median Filter for Acceleromter] """
    """======================================"""


    """======================================"""
    """   [Median Filter for Magnetometer]   """
    """======================================"""

    # Cycle the table
    for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
        mag_medianTable1X[x] = mag_medianTable1X[x-1]
        mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
        mag_medianTable1Z[x] = mag_medianTable1Z[x-1]

    # Insert the latest values
    mag_medianTable1X[0] = MAGx
    mag_medianTable1Y[0] = MAGy
    mag_medianTable1Z[0] = MAGz

    # Copy the tables
    mag_medianTable2X = mag_medianTable1X[:]
    mag_medianTable2Y = mag_medianTable1Y[:]
    mag_medianTable2Z = mag_medianTable1Z[:]

    # Sort table 2
    mag_medianTable2X.sort()
    mag_medianTable2Y.sort()
    mag_medianTable2Z.sort()

    # The middle value is the value we are interested in
    MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)]
    MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)]
    MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)]

    """======================================"""
    """ [End Median Filter for Magnetometer] """
    """======================================"""

    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN
    rate_gyr_z =  GYRz * G_GAIN

    # Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP
    gyroZangle+=rate_gyr_z*LP

    # Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

    # Change the rotation value of the accelerometer to -/+ 180 and
    # Move the Y axis '0' point to up.  This makes it easier to read.
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0

    # Complementary filter used to combine the accelerometer and gyro values.
    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

    # Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)

    # Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI
    if heading < 0:
        heading += 360

    """==============================="""
    """      [Tilt Compensation]      """
    """==============================="""

    #Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

    # Calculate pitch and roll
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    # X & Y Compensation
    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
    magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

    # Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360

    """==============================="""
    """    [End Tilt Compensation]    """
    """==============================="""

    return heading, tiltCompensatedHeading


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


def interpolate_vector(vectors):
    """
    This function takes a list of 41 tuples ('vectors'), where each tuple contains an angle and a distance.
    It interpolates the zero values for the distances and returns the modified list.
    
    Parameters:
    vectors (list of tuples): A list of tuples where the first value is the angle and the second is the distance.
    
    Returns:
    list of tuples: A modified list with interpolated values for distances that were originally zero.
    """
    # Extract the angles and distances from the input list
    angles = [angle for angle, distance in vectors]
    distances = [distance for angle, distance in vectors]
    
    # Count the number of non-zero distances
    non_zero_count = sum(1 for distance in distances if distance != 0)
    
    # Ensure that there are at least 6 non-zero distances
    if non_zero_count < 6:
        return vectors, -1
    
    # Identify the indices where distances are zero
    zero_indices = [i for i, distance in enumerate(distances) if distance == 0]
    
    # Extract the non-zero values
    non_zero_angles = [angles[i] for i, distance in enumerate(distances) if distance != 0]
    non_zero_distances = [distance for distance in distances if distance != 0]
    
    # Perform linear interpolation for the zero values
    interpolation_function = interpolate.interp1d(non_zero_angles, non_zero_distances, kind='linear', fill_value="extrapolate")
    
    # Replace the zero distances with the interpolated values
    for i in zero_indices:
        distances[i] = float(interpolation_function(angles[i]))
    
    # Recombine the angles and (interpolated) distances into tuples
    vectors = [(angles[i], distances[i]) for i in range(len(vectors))]
    
    return vectors, 0


def average_of_angles(vectors, start_angle, end_angle):
    """
    This function calculates the average value of angles between the given start and end angle (inclusive).
    
    Args:
    vectors (list of tuples): A list of tuples where the first element is the angle and the second is the value.
    start_angle (int): The starting angle of the range.
    end_angle (int): The ending angle of the range.
    
    Returns:
    float: The average value of the filtered angles.
    """
    # Filter the vectors where the angle is between start_angle and end_angle
    filtered_values = [value for angle, value in vectors if start_angle <= angle <= end_angle]
    return np.mean(filtered_values)


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
        else:
            pass  # for now
    # lh_vectors = interpolate_vector(lh_vectors)
    rh_vectors, rh_flag = interpolate_vector(rh_vectors)
    fw_vectors, fw_flag = interpolate_vector(fw_vectors)
    if rh_flag == 0:
        #print(rh_vectors)
        #print("\n")
        rh_avg = average_of_angles(rh_vectors, 170, 190)
        print(rh_avg)
        if rh_avg > 1750:
            print("Turn Right!")
        #print("\n\n")
        
    if fw_flag == 0:
        #print(fw_vectors)
        #print("\n")
        fw_avg = average_of_angles(fw_vectors, 80, 100)
        print(fw_avg)
        if fw_avg < 750:
            print("Dead End Found!")
        print("\n\n")

    # """ Target Angle """
    # # Get the longest string of zeros array (the angle we want to be)
    # zero_length, zero_start, zero_end = find_longest_string_of_zeros(fw_vectors)
    # largest_sum_index = -1
    # max_sum = 0
    # brake = 'F'
    # target_distance = 0
    # if zero_length < 5:
    #     print("!!Non Zero Target Selected!!")
    #     for i in range(len(fw_vectors) -5):
    #         current_sum = sum(fw_vectors[j][1] for j in range(i, i + 5))
    #         if current_sum > max_sum:
    #             max_sum = current_sum
    #             largest_sum_index = i
    #             target_distance = max_sum / 5
    #     if target_distance > 1250:
    #         target_angle = fw_vectors[largest_sum_index + 2][0]
    #     else:
    #         brake = 'N'
    # else:
    #     target_angle = int((zero_start + zero_end) / 2)

    # percent_ang = 0
    # fw_error = (target_angle - 90)
    # fw_deadband = 2
    
    # """ Proportion """
    # # Check if fw error is greater than deadband and set percent ang if so
    # if fw_error > 90 + fw_deadband or fw_error < 90 - fw_deadband:
    #     percent_ang = abs(fw_error) * Kfw
    #     percent_ang = min(0.6, percent_ang)
    #     if target_angle < 90:
    #         percent_ang = -percent_ang
    #     fw_ang = percent_ang

    # """ Integral """
    # # If Pico Is Ready, Enable Integral Term
    # if pico_rdy == 1:
    #     if target_angle < 90:
    #         fw_integral += fw_error * Ki
    #         fw_integral = max(-0.3, fw_integral)
    #     else:
    #         fw_integral += fw_error * Ki
    #         fw_integral = min(0.3, fw_integral)
    # percent_ang += fw_integral

    # """ Obstacle Avoidance """
    # obstacle_avoid_tol = 500
    # obstacle_ang = 0
    # angle_left = -1
    # angle_right = -1
    # dist_short_l = obstacle_avoid_tol + 1
    # dist_short_r = obstacle_avoid_tol + 1
    # for (angle, distance) in fw_vectors[zero_start-5: zero_start]:
    #     if distance <= obstacle_avoid_tol and distance < dist_short_l and distance > 0:
    #         angle_left = angle
    #         dist_short_l = distance
    # if(dist_short_l < obstacle_avoid_tol):
    #     print(f"Close Wall Detected Left at Ang: {angle_left} Dist: {dist_short_l}")
    
    # for (angle, distance) in fw_vectors[zero_end: zero_end + 5]:
    #     if distance <= obstacle_avoid_tol and distance < dist_short_r and distance > 0:
    #         angle_right = angle
    #         dist_short_r = distance

    # if(dist_short_r < obstacle_avoid_tol):
    #     print(f"Close Wall Detected Right at Ang: {angle_right} Dist: {dist_short_r}")
    
    # # Krabby Patty Secret Formula
    # if(dist_short_r < dist_short_l):
    #     obstacle_ang = 0.2 + (-0.05 / 350) * (dist_short_r - 150)
    #     percent_ang += obstacle_ang
    # elif(dist_short_r > dist_short_l):
    #     obstacle_ang = 0.2 + (-0.05 / 350) * (dist_short_l - 150) 
    #     percent_ang -= obstacle_ang
    
    # print(f"T Ang: {target_angle} T Dist: {target_distance}\n+ Fw P%: {round(fw_ang, 3)}  Fw I%: {round(fw_integral, 3)}  Ob %: {round(obstacle_ang, 3)}")
    
    # # Determine Direction to Turn
    # if percent_ang > 0:
    #     direction = 'R'
    # elif percent_ang < 0:
    #     direction = 'L'
    # else:
    #     direction = 'N'
    
    # print(f"=== [Total % {round(percent_ang, 3)} Dir {direction} Motor Dir {brake}] ===\n\n")
    # return direction, abs(percent_ang), brake


def process_data(data, headings):
    global tmp_cnt
    global UART_Rdy
    global travel_distance
    if UART_Rdy != 1:
        travel_distance = float(UART_Rdy)
    data_json = {
        "scan_data" : data,
    }
    heading, tilt_comp_heading = headings
    # For Mapping do rh_vector and lh_vector but with 360 measurements (and 0's in the non ideals)
    UART_Rdy = 0
    travel_distance = 0
    print(f"Heading {heading}, Tilt Comp Heading {tilt_comp_heading}")
    PID_control(data)
    #direction, percent_ang ,brake = PID_control(data)
    #write_pid_ctrl(direction, percent_ang, brake)
    #json.dumps(data_json)
    #requests.post('http://10.42.0.61:8069', json=data_json)

""" [Main Function] """
if True:
    # Setup Interrupt Handler (what is bouncetime?)
    GPIO.output(UART_PI_2_PICO_PIN, False)
    GPIO.output(PICO_DISABLE_PIN, False)
    GPIO.add_event_detect(UART_RDY_PIN, GPIO.RISING, callback=uart_irq_handler, bouncetime=200)
    GPIO.add_event_detect(PICO_RDY_PIN, GPIO.RISING, callback=pico_rdy_irq_handler, bouncetime=200)
    try:
        print("===== [pi_main.py] =====\n")
        print(" Starting...\n\n")
        time.sleep(0.1)
        print("=== [Connecting to IMU] ===")
        IMU.detectIMU()     #Detect if BerryIMU is connected.
        if(IMU.BerryIMUversion == 99):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
        prev_time = datetime.datetime.now()
        print("=== [Connecting to LiDAR Device] ===")
        for scan in lidar.iter_scans():
            for(quality, angle, distance) in scan:
                scan_data[min([359, math.floor(angle)])] = distance
            process_data(scan_data, compass_read())
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
