"""
\file       pico_driver.py
\brief      Driver code for deploying Pico code from the RPi and running it

\authors    Corbin Warmbier  |  corbinwarmbier@gmail.com

\date       Initial: 07/13/24  |  Last: 07/13/24
"""
import os
import sys

filename = "pico/pico_main.py"
flash_cmd = "ampy --port /dev/ttyACM0"

if len(sys.argv) == 2:
    if os.path.isfile(sys.argv[1]):
        filename = sys.argv[1]
    else:
        print(f"Bad argument received {sys.argv[1]}, file does not exist\nDefaulting to {filename}...\n\n")
print("===== [ Flashing Pico ] =====\n")
print(f"Program: {filename}")
os.system(flash_cmd + " put " + filename)
os.system(flash_cmd + " run " + filename)
print("===== [ Complete ] =====\n")