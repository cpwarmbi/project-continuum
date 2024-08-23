"""
\file       pico_main.py
\brief      Pico uC code for motor control and sensor interfacing.
            Interfaces w/ RPi for motor control instructions

\authors    Corbin Warmbier | corbinwarmbier@gmail.com

\date       Initial: 07/13/24  |  Last: 08/23/24
"""

""" [Imports] """
from machine import Pin, PWM, Timer, UART
import neopixel
import time
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

""" [Initialization] """
# Communication Lines
uart = UART(0, 115200, tx=Pin(0), rx=Pin(1), timeout=2)  # Initialize UART communication over USB
pico2pi = Pin(2, Pin.OUT)                                # Pico to Pi interrupt pin (Tx pin sending data)
pi2pico = Pin(3, Pin.IN, Pin.PULL_DOWN)                  # Pi to Pico interrupt pin (Rx pin receiving data)
disable = Pin(4, Pin.IN, Pin.PULL_DOWN)                  # Disable message sent from Pi to Pico
pico_rdy = Pin(5, Pin.OUT)                               # Enable message sent from Pico to Pi

# Ultrasonic Sensors
us_sensors = {
    'FL': {'trig': 6, 'echo': 7},
    'FR': {'trig': 8, 'echo': 9},
    'BR': {'trig': 10, 'echo': 11},
    'BL': {'trig': 14, 'echo': 15}
}

for location, sensor in us_sensors.items():
    sensor['trig_pin'] = Pin(sensor['trig'], Pin.OUT)
    sensor['echo_pin'] = Pin(sensor['echo'], Pin.IN)

# Motor Setup
# DC Motor
Motor_INA = Pin(16, Pin.OUT)                             
Motor_INB = Pin(17, Pin.OUT)
Motor_PWM = Pin(18, Pin.OUT)
Motor_PWM = PWM(Motor_PWM, freq = MOTOR_FREQ)

# Servo Motor
Servo_PWM = Pin(20, Pin.OUT)
Servo_PWM = PWM(Servo_PWM, freq = SERVO_FREQ)

# LED Initialization
led_strip = neopixel.NeoPixel(Pin(27), LED_COUNT)
led_timer = Timer()
led_counter = 0
turn_color = ''

# Init Globals
run = 1
pid_dir = 'N'
pid_ang = 0
pid_brake = 'F'
start_time = {}
distances = {}

# Initialize the pins and interrupts
for location, sensor in us_sensors.items():
    sensor['trig_pin'] = Pin(sensor['trig'], Pin.OUT)
    sensor['echo_pin'] = Pin(sensor['echo'], Pin.IN)
    sensor['echo_pin'].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=lambda pin, loc=location: echo_callback(pin, loc))
    start_time[location] = 0
    distances[location] = None

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
    print(f"= Date: 08/23/24                             =\n")
    time.sleep(0.3)
    print("= Praying to 100,000 Indian Gods...           =\n")
    time.sleep(0.3)
    print("= Sacrificing 50 goats...                     =\n")
    time.sleep(0.3)
    print("= Message sent, program starting              =\n")
    print("===============================================\n")
    time.sleep(0.3)  # The header must be seen...

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


def send_trigger(trig_pin):
    """
    Sends a trigger pulse to the ultrasonic sensor to initiate distance measurement.
    
    :param trig_pin <Pin>: The GPIO pin configured as the trigger pin for the ultrasonic sensor.
    :return: none
    """
    trig_pin.low()  # Ensure the trigger pin is low
    time.sleep_us(2)  # Wait for 2 microseconds
    trig_pin.high()  # Set the trigger pin high
    time.sleep_us(10)  # Wait for 10 microseconds to send a pulse
    trig_pin.low()  # Set the trigger pin low again


def echo_callback(pin, location):
    """
    Handles the interrupt for the ultrasonic sensor's echo pin.
    Calculates the distance based on the time taken for the echo to return.

    :param pin <Pin>: The GPIO pin that triggered the interrupt (echo pin).
    :param location <str>: The location identifier of the ultrasonic sensor (e.g., 'FL').
    :return: none
    """
    global start_time, distances
    if pin.value():  # Rising edge detected
        start_time[location] = time.ticks_us()  # Record the current time in microseconds
    else:  # Falling edge detected
        duration = time.ticks_diff(time.ticks_us(), start_time[location])  # Calculate the time difference
        distances[location] = (duration * 0.0343) / 2  # Convert time to distance in centimeters


def trigger_all_sensors():
    """
    Sends a trigger pulse to all configured ultrasonic sensors.
    
    :return: none
    """
    for sensor in us_sensors.values():
        send_trigger(sensor['trig_pin'])  # Trigger each ultrasonic sensor


def car_init():
    """
    Initializes all motors to neutral and brakes
    """
    set_motor_dir('B')
    set_motor_spd(0)
    set_servo('N')
    time.sleep(0.2)


def car_stop():
    global turn_color
    turn_color == 'P'
    car_init()


def disable_irq_handler(edge_type):
    global run
    run = 0


def send_sensor_data(speed):
    print("Sending Sensor data to Pi")
    speed = speed +'\n'
    pico2pi.value(1)
    if uart.write(speed.encode('utf-8')) < 0:
        print("Error sending UART message to pi")
    uart.flush()
    pico2pi.value(0)


def rx_irq_handler(edge_type):
    global pid_dir
    global pid_ang
    global pid_brake
    time.sleep(0.03)
    message = uart.readline().decode('utf-8')
    pid_dir, percent_ang, pid_brake = (message.strip('\n')).split(' ')
    pid_ang = float(percent_ang)


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
    # IRQ Setup
    disable.irq(trigger = Pin.IRQ_RISING, handler = disable_irq_handler)
    led_timer.init(mode = Timer.PERIODIC, freq= LED_FREQ, callback = led_handler)
    pi2pico.irq(trigger = Pin.IRQ_RISING, handler = rx_irq_handler)
    for name, sensor in us_sensors.items():
        sensor['echo_pin'].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=lambda pin, loc=name: echo_callback(pin, loc))
        start_time[name] = 0
        distances[name] = None

    # Init
    pico_rdy.value(0)
    program_header()
    car_init()

    while run:
        pico_rdy.value(1)
        trigger_all_sensors()
        time.sleep(0.1)  # Allow time for all echoes to return
        # Print distances from ultrasonic sensors
        for location, distance in distances.items():
            if distance is not None:
                print(f"Distance to {location}: {distance:.2f} cm")
            else:
                print(f"Distance to {location}: Out of range or no reading")

        # Set Motor to Forward for 30%
        # set_servo(pid_dir, pid_ang)
        # set_motor_dir(pid_brake)
        # set_motor_spd(0.21)
        # time.sleep(0.2)
        # set_motor_dir('F')
        # set_motor_spd(0.2)
        time.sleep(0.2)

    pico_rdy.value(0)
    car_stop()
