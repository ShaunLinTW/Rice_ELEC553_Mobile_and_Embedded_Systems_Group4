'''
ELEC 424/553
Project 3
Authors: Eric Lin(el38), Shaun Lin(hl116), Yen-Yu Chien (yc185), Saif Khan (sbk7)
'''
import time
import math

# stop: 7.5
# forward min: 8.0
# forward max: 9.0
# ESC motor speed function
def ESC(percentage):
    # P9_14 - Speed/ESC
    with open('/dev/bone/pwm/1/a/period', 'w') as filetowrite:
        filetowrite.write('20000000')
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(percentage/100*20000000)))
    with open('/dev/bone/pwm/1/a/enable', 'w') as filetowrite:
        filetowrite.write('1')
    return

# turn left max: 11
# turn right max: 3
# steering angle of the servo function
def Servo(percentage):
    # P9_16 - Steering
    with open('/dev/bone/pwm/1/b/period', 'w') as filetowrite:
        filetowrite.write('20000000')
    with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(percentage/100*20000000)))
    with open('/dev/bone/pwm/1/b/enable', 'w') as filetowrite:
        filetowrite.write('1')  
    return

# Initialize the servo
servo_percentage = 7.5
Servo(servo_percentage)
# Initialize the ESC motor
motor_percentage = 7.5
ESC(motor_percentage)
time.sleep(0.5)

# Start the motor
for x in range(75, 91):
    ESC(x/10)
    time.sleep(0.1)

# Stop the motor
ESC(7.5)
time.sleep(0.5) 

# Turn from left to right
for x in range(3, 11):
    Servo(x)
    time.sleep(0.1)

# Turn Servo back to center
Servo(7.5)
time.sleep(0.5)