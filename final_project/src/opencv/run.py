'''
ELEC 424/553
Final Project
Authors: Eric Lin(el38), Shaun Lin(hl116), Yen-Yu Chien (yc185), Saif Khan (sbk7)
'''

# Import libraries
import time
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math
from collections import Counter
#from yolo_utils import *

# Code references from https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
# Code references from https://www.hackster.io/covid-debuff/covid-debuff-semi-autonomous-rc-car-platform-75b072
# Many thanks to team COVID Debuff

# Convert the image to HSV format
def convert_to_HSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #   cv2.imshow("HSV",hsv)
    return hsv


# Detect the edges using canny method
def detect_edges(frame):
    # lower_blue = np.array([100, 43, 46], dtype = "uint8") # lower limit of blue color
    # upper_blue = np.array([124, 255, 255], dtype="uint8") # upper limit of blue color
    lower_blue = np.array([70, 90, 0], dtype="uint8")  # lower limit of blue color
    upper_blue = np.array([160, 255, 255], dtype="uint8")  # upper limit of blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # this mask will filter out everything but blue

    # detect edges
    edges = cv2.Canny(mask, 50, 200)
    # cv2.imshow("edges",edges)
    return edges


# Capture the bottom half of the image
def region_of_interest(edges):
    height, width = edges.shape  # extract the height and width of the edges frame
    mask = np.zeros_like(edges)  # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)  # fill the polygon with blue color
    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("roi",cropped_edges)
    return cropped_edges

# Convert Envelopes to Regions
def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 15
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=50) #maxLineGap change from 0 to 50
    return line_segments


# Slope calculation
def average_slope_intercept(frame, line_segments): #line segments stores HoughLines
    lane_lines = []

    if line_segments is None:
        # print("no line segment detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3
    #boundary = 1 / 2
    left_average = 0
    right_average = 0
    left_cnt = 0
    right_cnt = 0

    left_region_boundary = width * (1 - boundary) # set boundary in x-axis
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                # print("sKi_stpping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                right_average = right_average + slope
                right_cnt = right_cnt + 1

                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                left_average = left_average + slope
                left_cnt = left_cnt + 1

                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average)) # make_points is defined in the next function

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane
    # all coordinate points are in pixels
    return lane_lines


# Draw lines
def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]


# Display lines
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):  # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


# Calculate the steering angles
def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:  # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0]  # extract left x2 from lane_lines array   # x2 is in the upper side of the camera
        _, _, right_x2, _ = lane_lines[1][0]  # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid # the offset is the offset the original middle line
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:  # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0] # extract the x1 and x2 from the only line
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:  # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset) # use the offset to calculate the offset radian from the original middle line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle


# Draw the heading line
def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi # steeting_angle is by degree, so now to transfer
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian)) # = (x1 - (height/2)/tangent )
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

#======================================= 20231201 ===========================================

# Modify the speed of the motor
def modify_ESC(percentage):
    # P9_14 - Speed/ESC
    with open('/dev/bone/pwm/1/a/period', 'w') as filetowrite:
        filetowrite.write('20000000')
    with open('/dev/bone/pwm/1/a/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(percentage / 100 * 20000000)))
    with open('/dev/bone/pwm/1/a/enable', 'w') as filetowrite:
        filetowrite.write('1')
    return


# Modify the steering angle of the servo
def modify_Servo(percentage):
    # P9_16 - Steering
    with open('/dev/bone/pwm/1/b/period', 'w') as filetowrite:
        filetowrite.write('20000000')
    with open('/dev/bone/pwm/1/b/duty_cycle', 'w') as filetowrite:
        filetowrite.write(str(int(percentage / 100 * 20000000)))
    with open('/dev/bone/pwm/1/b/enable', 'w') as filetowrite:
        filetowrite.write('1')
    return


# read the speed from the encoder
# def read_encoder():
    # with open('/sys/module/yg77_device/parameters/speed', 'r') as filetoread:
    #     speed = filetoread.read()
    # return speed


# Detect the red edges for red box detection
def detect_red_edges(frame):
    # lower_blue = np.array([150, 30, 30], dtype="uint8")  # lower limit of red color
    # upper_blue = np.array([180, 255, 255], dtype="uint8")  # upper limit of red color

    lower_blue = np.array([0, 30, 166], dtype="uint8")  # lower limit of pink color
    upper_blue = np.array([179, 68, 246], dtype="uint8")  # upper limit of pink color

    mask = cv2.inRange(hsv, lower_blue, upper_blue)  # this mask will filter out everything but red
    return mask


# # Initialize the servo
servo_percentage = 7.5
modify_Servo(servo_percentage)
# # Initialize the motor
modify_ESC(7.5)
time.sleep(0.5)
# # Start the motor
modify_ESC(8.0)
# Initialize the camera
#video = cv2.VideoCapture(1,cv2.CAP_DSHOW)
video = cv2.VideoCapture(2)
# Set the resolution of the camera
video.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

# Status vairables
last_error_st = 0
# last_error_sp = 0
last_time = 0
# last_encoder = 0
# PID parameters
Kp_st = 0.1944
Ki_st = 0.10935
Kd_st = 0.03888 * 1.5

# Kp_sp = 0.001944
# Ki_sp = 0.0
# Kd_sp = 0.0
# Default speed
default_speed = 8.0

# Records for graphs
iter_cnt = 0
T = []
steer_error = []
speed_error = []
steer_P = []
steer_I = []
steer_D = []
speed = []
steer_pwm_duty = []
speed_pwm_duty = []

# Sign variables
end_flag = 0
target_speed = 4e-07

while True:
    # Reading from the encoder
    # encoder = read_encoder()
    # Read from camera
    ret, frame = video.read()
    # print the current frame number
    frames_num = video.get(cv2.CAP_PROP_POS_FRAMES)
    if frame is None:
   	 print("Error: Unable to load the image.")

    # Process the images
    hsv = convert_to_HSV(frame)
    edges = detect_edges(hsv)
    red_edges = detect_red_edges(hsv)
    red_edges = region_of_interest(red_edges)
    line_segments = detect_line_segments(edges)
    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image, steering_angle)
    print(steering_angle)
    # Detect the red box
    sum = Counter(red_edges.flatten())
    # for key, value in sum.items():
    #     if (key == 255):
    #         if (value > 100 and abs(90 - steering_angle) < 50):
    #             end_flag = 1
    #             print("RED DETECTED")

    # # Stop Sign Dectection starts here
    # # Code reference from https://github.com/fredotran/traffic-sign-detector-yolov4
    # # test our function read_classes
    # img_file = './data/obj.names'
    # classNames = read_classes(img_file)
    # # print("Classes' names :\n", classNames)
    # # load the model config and weights
    # modelConfig_path = './cfg/yolov4-rds.cfg'
    # modelWeights_path = './weights/yolov4-rds_best_2000.weights'
    # # read the model cfg and weights with the cv2 DNN module
    # neural_net = cv2.dnn.readNetFromDarknet(modelConfig_path, modelWeights_path)
    # # set the preferable Backend to GPU
    # neural_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    # neural_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    # # defining the input frame resolution for the neural network to process
    # network = neural_net
    # height, width = 160, 160
    # # confidence and non-max suppression threshold for this YoloV3 version
    # confidenceThreshold = 0.5
    # nmsThreshold = 0.2
    # # load the image
    # img = frame
    # # using convert_to_blob function :
    # outputs = convert_to_blob(img, network, height, width)
    # # apply object detection on the video file
    # bounding_boxes, class_objects, confidence_probs = object_detection(outputs, img, confidenceThreshold)
    # # perform non-max suppression
    # indices = nms_bbox(bounding_boxes, confidence_probs, confidenceThreshold, nmsThreshold)
    # # draw the boxes
    # res = box_drawing(img, indices, bounding_boxes, class_objects, confidence_probs, classNames, color=(255, 0, 0),
    #                   thickness=2)
    # # If the Stop Sign detected
    # if (res == 1):
    #     print("STOP SIGN DETECTION")
    #     modify_ESC(7.5)
    #     time.sleep(5)
    #     modify_ESC(default_speed)
    #     # Update the time
    now = time.time()
    dt = now - last_time
    #
    # # Steer PID
    error_st = 90 - steering_angle
    error_diff_st = (error_st - last_error_st) / dt
    if (iter_cnt != 0):
        error_int_st = (error_st - last_error_st) * dt
    else:
        error_int_st = 0
    last_error_st = error_st
    servo_percentage = servo_percentage + Kp_st * error_st + Kd_st * error_diff_st + Ki_st * error_int_st
    # Correct the overshoot
    if (servo_percentage > 8.8):
        servo_percentage = 8.8
    elif (servo_percentage < 6.3):
        servo_percentage = 6.3
    # modify the PWM duty of servo
    modify_Servo(servo_percentage)
    # -------- we don't have encoder, so we don't need to do the speed PID --------
    # Speed PID
    # error_sp = int(encoder) - target_speed
    # error_diff_sp = (error_sp - last_error_sp) / dt
    # if (iter_cnt != 0):
        # error_int_sp = (error_sp - last_error_sp) * dt
    # else:
        # error_int_sp = 0
    # last_error_sp = error_sp
    # default_speed = default_speed - Kp_sp * error_sp + Kd_sp * error_diff_sp + Ki_sp * error_int_sp
    # Correct the overshoot
    # if (default_speed > 8.30):
        # default_speed = 8.30
    # elif (default_speed < 8.20):
        # default_speed = 8.20
    # modify the PWM duty of motor
    modify_ESC(default_speed)
    # Show the image
    # cv2.imshow('original', heading_image)
    key = cv2.waitKey(10)
    # save each frame to a png to make a video '/home/debian/Group4/src/opencv/frames/snap%s.png'
    cv2.imwrite('/home/debian/Group4/src/opencv/frames/snap%s.png' % frames_num, heading_image)


    # # Record the variables
    # T = np.append(T, [iter_cnt])
    # steer_error = np.append(steer_error, [error_st / 100])
    # # speed_error = np.append(speed_error, [1 / error_sp * 1e7])
    # steer_P = np.append(steer_P, [Kp_st * error_st])
    # steer_I = np.append(steer_I, [Kd_st * error_diff_st])
    # steer_D = np.append(steer_D, [Ki_st * error_int_st])
    # # speed_pwm_duty = np.append(speed_pwm_duty, [default_speed])
    # # speed = np.append(speed, [error_sp])
    # steer_pwm_duty = np.append(steer_pwm_duty, [servo_percentage])
    # iter_cnt = iter_cnt + 1
    # if (end_flag == 1):
    #     break

# Stop the car
# modify_ESC(7.5)

# Plot the first graph
# plt.figure()
# plt.plot(T, steer_error, color='orange', linewidth=1.0, linestyle='solid', label='Error')
# plt.plot(T, steer_P, color='red', linewidth=1.0, linestyle='solid', label='Proportional Response')
# plt.plot(T, steer_I, color='blue', linewidth=1.0, linestyle='solid', label='Derivative Response')
# plt.plot(T, steer_D, color='green', linewidth=1.0, linestyle='solid', label='Integral Response')
# plt.legend(loc='upper right')
# plt.title('PID Responses vs. Frame Number')
# plt.xlabel('Frame')
# plt.ylabel('PID Responses')
# plt.grid()

# Plot the second graph
# plt.figure()
# plt.plot(T, speed_error, color='red', linewidth=1.0, linestyle='solid', label='Speed Error')
# plt.plot(T, steer_error, color='orange', linewidth=1.0, linestyle='solid', label='Steer Error')
# plt.plot(T, speed_pwm_duty, color='blue', linewidth=1.0, linestyle='solid', label='Speed PWM')
# plt.plot(T, steer_pwm_duty, color='green', linewidth=1.0, linestyle='solid', label='Steer PWM')
# plt.legend(loc='upper right')
# plt.title('PWM and Error vs. Frame Number')
# plt.xlabel('Frame')
# plt.ylabel('Values')
# plt.ylim((0, 11.5))
# plt.grid()
# Do the plot
# plt.show()
