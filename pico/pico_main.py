"""
\file       pico_main.py
\brief      Pico uC code for motor control and sensor interfacing.
            Interfaces w/ RPi for motor control instructions

\authors    Corbin Warmbier | corbinwarmbier@gmail.com

\date       Initial: 07/13/24  |  Last: 07/13/24
"""

""" [Imports] """
from machine import ADC, Pin, PWM, Timer, UART
import neopixel
from time import sleep
from math import pi


# ========================================= #
#      === [Declarations & Init] ===        #
# ========================================= #

""" [Constants] """
MOTOR_FREQ = 5000       # 5kHz; optimal VNH freq
SERVO_FREQ = 100
MOTOR_SPD_MAX = 200000  # Based on MOTOR_FREQ ! Must Change if MOTOR_FREQ is modified
TIMER_FREQ = 4          # Hz
LED_FREQ = 5            # Hz
LED_COUNT = 44
COUNTS_TO_ROTATION = 3  # Number of counters per wheel rotation (aka # of tape pieces)

""" [Initialization] """
# DC Motor Pin Setup
Motor_PWM = Pin(13, Pin.OUT)
Motor_INA = Pin(14, Pin.OUT)
Motor_INB = Pin(15, Pin.OUT)
Motor_CS = ADC(Pin(28))  # Optional Current Sensing Pin; Analog Read

Servo_PWM = Pin(20, Pin.OUT)                # Servo Motor Pin Setup
Motor_Spd = Pin(22, Pin.IN, Pin.PULL_DOWN)  # Color Sensor for Tracking Speed Pin Setup
UART_Rdy = Pin(2, Pin.OUT)                  # Interrupt Pin to set High when ready to transmit UART data

# Initialize Pins for PWM and Set Frequency
Motor_PWM = PWM(Motor_PWM, freq = MOTOR_FREQ)
Servo_PWM = PWM(Servo_PWM, freq = SERVO_FREQ)

# LED Initialization
led_strip = neopixel.NeoPixel(Pin(9), LED_COUNT)
led_timer = Timer()
led_counter = 0
turn_color = ''

# Init UART Communication Lines
pi2pico = Pin(3, Pin.IN, Pin.PULL_DOWN)
uart = UART(0, 115200, tx=Pin(0), rx=Pin(1), timeout=2)

Disable = Pin(4, Pin.IN, Pin.PULL_DOWN)
Pico_Rdy = Pin(5, Pin.OUT)

# Init Globals
timer = Timer()
counter = 0
last_distance = 0
traveled_distance = 0
run = 1
pid_dir = 'N'
pid_ang = 0
pid_brake = 'F'

# ========================================= #
#         === [Local Functions] ===         #
# ========================================= #
def program_header():
    """
    Prints program header on startup
    :return: none
    """
    print("===============================================\n")
    print("=============== [EEC195 Team 5] ===============\n")
    print("===============================================\n")
    print("=                                             =\n")
    print("= Date: 05/17/2024                            =\n")
    sleep(0.3)
    print("= Praying to 100,000 Indian Gods...           =\n")
    sleep(0.3)
    print("= Sacrificing 50 goats...                     =\n")
    sleep(0.3)
    print("= Message sent, program starting              =\n")
    print("===============================================\n")
    sleep(0.3)  # The header must be seen...

def set_motor_dir(direction):
    """
    Sets motor to desired direction. If given invalid direction, motor direction
    will be unaffected.

    :param direction <char>: Sets motor to one of the 3 accepted settings:
        + 'F': Forward
        + 'R': Reverse
        + 'B': Brake
    :return: none
    """
    if direction == 'F' or direction == 'f':
        Motor_INA.value(1)
        Motor_INB.value(0)
    elif direction == 'R' or direction == 'r':
        Motor_INA.value(0)
        Motor_INB.value(1)
    elif direction == 'B' or direction == 'b':
        Motor_INA.value(0)
        Motor_INB.value(0)


def set_motor_spd(percent_spd):
    """
    Sets motor to desired speed as a percent of the MAXIMUM speed.
    !! set_motor_dir SHOULD be called before this function !!

    :param percent_spd <float>: Sets motor to desired speed. Acceptable Range (0.0 - 1.0)
    :return: none
    """
    if percent_spd < 0 or percent_spd > 1:
        return
    duty_cycle = MOTOR_SPD_MAX * percent_spd  # Convert Percent Speed to Duty Cycle in ns
    Motor_PWM.duty_ns(int(duty_cycle))


def set_servo(direction, percent_ang=0):
    global turn_color
    """
    Sets servo to desired direction and at the percentage of that direction.
    100% percent in a direction correlates to roughly ~ 45 degrees

    :param direction <char>: Sets servo to one of the 3 accepted settings:
        + 'R': Right
        + 'L': Left
        + 'N': Neutral
    :param percent_ang <float>: Turns to a percentage of the maximum angle. Range (0.0 - 1.0)
        * Note: percent_angle is not used in neutral setting. 1.0 = Max angle ~ 45 degrees
    :return: none
    """
    if percent_ang < 0 or percent_ang > 1:
        return
    if direction == 'R' or direction == 'r':
        # Range (1100000 -> 1500000)
        delta = 400000 * percent_ang
        turn_color = 'R'
        Servo_PWM.duty_ns(int(1500000 - delta))
    elif direction == 'L' or direction == 'l':
        # Range (1500000 -> 1900000)
        delta = 400000 * percent_ang
        Servo_PWM.duty_ns(int(1500000 + delta))
        turn_color = 'G'
    elif direction == 'N' or direction == 'n':
        Servo_PWM.duty_ns(1500000)
        turn_color = 'B'


def car_init():
    """
    Initializes all motors to neutral and brakes
    """
    set_motor_dir('N')
    set_motor_spd(0)
    set_servo('N')
    sleep(0.2)


def car_stop():
    global turn_color
    turn_color == 'P'
    timer.deinit()
    car_init()

def disable_irq_handler(edge_type):
    global run
    run = 0

def spd_irq_handler(edge_type):
    """
    Interrupt handler for speed sensor
    
    :param edge_type <Pin obj>: Falling or rising edge irq detection
    :return: none
    """
    global counter
    counter += 1

def send_sensor_data(speed):
    print("Sending Sensor data to Pi")
    speed = speed +'\n'
    UART_Rdy.value(1)
    if uart.write(speed.encode('utf-8')) < 0:
        print("Error sending UART message to pi")
    uart.flush()
    UART_Rdy.value(0)


def rx_irq_handler(edge_type):
    global pid_dir
    global pid_ang
    global pid_brake
    sleep(0.03)
    message = uart.readline().decode('utf-8')
    pid_dir, percent_ang, pid_brake = (message.strip('\n')).split(' ')
    pid_ang = float(percent_ang)

def spd_counter(timer):
    global counter
    global traveled_distance
    if counter != 0:
        distance = (counter / COUNTS_TO_ROTATION) * 4.1 * pi  # Distance formula 'Rotations * Wheel Dia * Pi = Inches'
        print(f'Traveled {distance} inches')
        traveled_distance += distance
        send_sensor_data(str(distance))
    counter = 0


# 4.1 inches (diameter of wheel)
# Determine RPM
# Equation: RPM * Wheel Diameter * Pi == Inches / Min
def get_distance():
    global last_distance
    global traveled_distance
    distance = traveled_distance - last_distance
    last_distance = traveled_distance
    print(f'Traveled {distance} inches since last func call')
    return distance

def led_handler(timer):
    global led_counter
    offset = led_counter % 11
    for i in range(0, LED_COUNT):
        intensity = ((23*(i+offset)) % 11)
        if intensity < 3:
            intensity = 0
        if turn_color == 'R':
            led_strip[i] = (intensity*2, 0, 0)
        elif turn_color == 'G':
            led_strip[i] = (0, intensity*2, 0)
        elif turn_color == 'N':
            led_strip[i] = (0,0,  intensity*2)
        elif turn_color == 'P':
            led_strip[i] = (0, 10*(led_counter %10), 0)
        else:
            led_strip[i] = (128*(led_counter %2), 0, 0)
    led_strip.write()
    led_counter += 1

# ========================================= #
#          === [Main Function] ===          #
# ========================================= #
if True:
    Pico_Rdy.value(0)
    led_timer.init(mode = Timer.PERIODIC, freq= LED_FREQ, callback = led_handler)
    # Init
    program_header()
    car_init()
    Motor_Spd.irq(trigger = Pin.IRQ_RISING, handler = spd_irq_handler)
    timer.init(mode = Timer.PERIODIC, freq = TIMER_FREQ, callback = spd_counter)
    
    pi2pico.irq(trigger = Pin.IRQ_RISING, handler = rx_irq_handler)
    Disable.irq(trigger = Pin.IRQ_RISING, handler = disable_irq_handler)
    # uart.irq(UART.RX_any, handler=rx_irq_handler)

    while run:
        Pico_Rdy.value(1)
        # Set Motor to Forward for 30%
        # set_servo(pid_dir, pid_ang)
        # set_motor_dir(pid_brake)
        # set_motor_spd(0.21)
        # sleep(0.2)
        set_motor_spd(0.2)
        sleep(0.2)

    Pico_Rdy.value(0)
    car_stop()
