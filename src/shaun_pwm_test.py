import time
import math

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
# ESC(8.0)
# time.sleep(0.5)

Servo(11)
time.sleep(3)

Servo(7.5)
time.sleep(0.5)
