# ADAPTED FROM https://github.com/EdjeElectronics/Train-and-Deploy-YOLO-Models/blob/main/yolo_detect.py

import os
import sys
import argparse
import glob
import time
import subprocess
import threading # Added for concurrent sensor reading

import cv2
import numpy as np
from ultralytics import YOLO
import os
import RPi.GPIO as GPIO
from collections import deque

# Global variables for thread communication
# These variables hold the last measured distances from the ultrasonic sensors.
global DISTANCE
global BOT_DISTANCE
DISTANCE = 400
BOT_DISTANCE = 400
from rpi_ws281x import *


#os.putenv('SDL_VIDEODRV', 'fbcon')
#os.putenv('SDL_FBDEV', '/dev/fb0')
#os.putenv('SDL_MOUSEDRV', 'dummy')
#os.putenv('SDL_MOUSEDEV', '/dev/null')
#os.putenv('DISPLAY', '')

# Start bluetooth speaker
subprocess.run('~/autopair', shell=True)

# ====================================
#         Motor Control Init
# ====================================
# Left servo pins
left_ina_pin = 5
left_inb_pin = 6
left_pwm_pin = 26

# Right servo pins
right_ina_pin = 17
right_inb_pin = 20
right_pwm_pin = 16

# GPIO Setup
pwm_freq = 50
duty_cycle = 0

GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
GPIO.setup(left_ina_pin, GPIO.OUT)
GPIO.setup(left_inb_pin, GPIO.OUT)
GPIO.setup(left_pwm_pin, GPIO.OUT)
GPIO.setup(right_ina_pin, GPIO.OUT)
GPIO.setup(right_inb_pin, GPIO.OUT)
GPIO.setup(right_pwm_pin, GPIO.OUT)

left_pwm = GPIO.PWM(left_pwm_pin, pwm_freq)
right_pwm = GPIO.PWM(right_pwm_pin, pwm_freq)

# ====================================
#       MOTOR CONTROL FUNCTIONS
# ====================================

def clockwise(pwm, ina_pin, inb_pin, speed):
    GPIO.output(ina_pin, GPIO.HIGH)
    GPIO.output(inb_pin, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def counter_clockwise(pwm, ina_pin, inb_pin, speed):
    GPIO.output(ina_pin, GPIO.LOW)
    GPIO.output(inb_pin, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def stop_motor(pwm, ina_pin, inb_pin):
    GPIO.output(ina_pin, GPIO.LOW)
    GPIO.output(inb_pin, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def move(left_speed, right_speed):
    right_speed = right_speed * (1-12/75)
        
    if left_speed > 0:
        clockwise(left_pwm, left_ina_pin, left_inb_pin, left_speed) # -12 because wheels misaligned/motor power mismatched so left motors is stronger/leans to left
    elif left_speed < 0:
        counter_clockwise(left_pwm, left_ina_pin, left_inb_pin, -(left_speed - 0))
    else:
        stop_motor(left_pwm, left_ina_pin, left_inb_pin)

    if right_speed > 0:
        clockwise(right_pwm, right_ina_pin, right_inb_pin, right_speed)
    elif right_speed < 0:
        counter_clockwise(right_pwm, right_ina_pin, right_inb_pin, -right_speed)
    else:
        stop_motor(right_pwm, right_ina_pin, right_inb_pin)

def stop():
    stop_motor(left_pwm, left_ina_pin, left_inb_pin)
    stop_motor(right_pwm, right_ina_pin, right_inb_pin)

# Initialize PWM
left_pwm.start(0)
right_pwm.start(0)

# ====================================
#        END MOTOR CONTROL SETUP
# ====================================

# ====================================
#       SPEECH CONTROL SETUP
# ====================================

def speak_espeak(text):
    """
    Synthesizes speech using eSpeak.
    """
    command = ["espeak", text]
    subprocess.Popen(command)

# ====================================
#     END SPEECH CONTROL SETUP
# ====================================

# ====================================
#       SOUND DETECTION SETUP
# ====================================

# Define the GPIO pin connected to the sound sensor's D0 pin
SOUND_SENSOR_PIN = 4 # Using BCM numbering for GPIO 4
spinMode = False
interuptEnable = False

# --- Setup ---
GPIO.setmode(GPIO.BCM) 
# Set the pin as an INPUT with a PULL-DOWN resistor, 
# as you requested and based on the assumption the sensor is active-HIGH.
GPIO.setup(SOUND_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

print("Sound Sensor Test Running (Interrupt Mode)")
print("Waiting for sound on GPIO 4...")
print("(Press Ctrl+C to exit)")

# --- Callback Function ---
# This function runs automatically whenever the event is detected
def sound_detected_callback(channel):
    """
    Callback function executed when a rising edge is detected on the GPIO pin.
    """
    # Note: We can add a debounce here to prevent multiple triggers from a single clap.
    global spinMode
    global interuptEnable
    if interuptEnable:
        spinMode = True
        current_time = time.strftime("%H:%M:%S")
        print(f"\n--- {current_time} --- ?? Sound Detected!")
    
    # You could add your action code here (e.g., logging, triggering an LED)


# --- Event Detection Setup ---

# 1. Add the event detection to the pin.
#    - GPIO.RISING: Trigger on the LOW-to-HIGH transition (sound detected). 
#    - bouncetime=200: Ignores subsequent triggers for 200 milliseconds,
#                      which prevents electrical noise from causing multiple detections.
GPIO.add_event_detect(
    SOUND_SENSOR_PIN, 
    GPIO.RISING, 
    callback=sound_detected_callback, 
    bouncetime=200
)


# ====================================
#     END SOUND DETECTION SETUP
# ===================================

# ====================================
#     ULTRASONIC SENSOR SETUP  
# ====================================

# Ultrasonic sensor pins
ECHO_PIN = 19
TRIG_PIN = 13

BOT_ECHO_PIN = 22
BOT_TRIG_PIN = 27


# Set up GPIO for ultrasonic sensors
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.setup(BOT_TRIG_PIN, GPIO.OUT)
GPIO.setup(BOT_ECHO_PIN, GPIO.IN)

# Ensure triggers are low initially
GPIO.output(TRIG_PIN, False)
GPIO.output(BOT_TRIG_PIN, False)
time.sleep(0.5) # Allow time for sensors to settle

# Constants for distance calculation
SPEED_OF_SOUND = 34300 # cm/s
STOP_DISTANCE_CM = 10 # Distance at which the robot should stop for obstacle avoidance

def get_distance_cm(TRIG_PIN, ECHO_PIN):
    """
    Measures the distance in centimeters using the ultrasonic sensor.
    """
    # Trigger pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001) # 10 microsecond pulse
    GPIO.output(TRIG_PIN, False)

    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for the echo pin to go HIGH (start of pulse)
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        # Timeout to prevent infinite loop
        if pulse_start - timeout_start > 0.04: 
            return 400 

    # Wait for the echo pin to go LOW (end of pulse)
    timeout_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        # Timeout to prevent infinite loop
        if pulse_end - timeout_end > 0.04:
            return 400

    # Calculate duration and distance
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * SPEED_OF_SOUND) / 2
    
    # Return distance, clamped to a reasonable maximum
    return min(distance, 400) 

def move_back():
    global DISTANCE
    stop()
    speak_espeak('ahhhh too close')
    print(f"Obstacle detected! Prioritizing avoidance. Dist {DISTANCE}")
    
    # Move backwards briefly
    move(-60, -60)
    time.sleep(1)
    stop()
    time.sleep(1)

class UltrasonicThread(threading.Thread):
    def __init__(self, threadID, name, trig, echo, bot_trig, bot_echo):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.trig = trig
        self.echo = echo
        self.bot_trig = bot_trig
        self.bot_echo = bot_echo
        self._running = True

    def run(self):
        print(f"Starting {self.name}")
        while self._running:
            global DISTANCE, BOT_DISTANCE
            # Read distances from both sensors
            DISTANCE = get_distance_cm(self.trig, self.echo)
            BOT_DISTANCE = get_distance_cm(self.bot_trig, self.bot_echo)

            if DISTANCE < STOP_DISTANCE_CM or BOT_DISTANCE < STOP_DISTANCE_CM:
                move_back()
                print("moving back")

            # Wait a short time before the next reading (60ms = ~16Hz update rate)
            time.sleep(0.06) 
        print(f"Exiting {self.name}")

    def stop(self):
        self._running = False

# class AvoidanceThread(threading.Thread):
#     def __init__(self, threadID, name):
#         threading.Thread.__init__(self)
#         self.threadID = threadID
#         self.name = name
#         self._running = True

#     def run(self):
#         print(f"Starting {self.name}")
#         while self._running:
#             global DISTANCE, RIGHT_DISTANCE
#             # Check distances and perform avoidance if necessary
#             #print(f"L Dist: {DISTANCE:.1f}cm, R Dist: {RIGHT_DISTANCE:.1f}cm")
#             if DISTANCE < STOP_DISTANCE_CM or RIGHT_DISTANCE < STOP_DISTANCE_CM:
#                 stop()
#                 speak_espeak('ahhhh too close')
#                 print("Obstacle detected! Prioritizing avoidance.")
                
#                 # Move backwards briefly
#                 move(-50, -50)
#                 time.sleep(1)
#                 stop()
                
#                 # Spin away from the obstacle
#                 if DISTANCE < RIGHT_DISTANCE:
#                     # Obstacle on the left, spin right
#                     move(40, -40) 
#                 else:
#                     # Obstacle on the right or both sides, spin left
#                     move(-40, 40)
                
#                 time.sleep(0.2)
#                 # Wait a short time before the next check
#             time.sleep(0.06) 
#         print(f"Exiting {self.name}")

#     def stop(self):
#         self._running = False

# Instantiate and start the ultrasonic thread
ultrasonic_thread = UltrasonicThread(
    1, "Ultrasonic-Reader", 
    TRIG_PIN, ECHO_PIN,
    BOT_TRIG_PIN, BOT_ECHO_PIN,
)
ultrasonic_thread.start()

# start the avoidance thread
# avoidance_thread = AvoidanceThread(2, "Obstacle-Avoidance")
# avoidance_thread.start()

# ====================================
#     END ULTRASONIC SENSOR SETUP  
# ====================================


# Define and parse user input arguments

parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--source', help='Image source, can be image file ("test.jpg"), \
                    image folder ("test_dir"), video file ("testvid.mp4"), index of USB camera ("usb0"), or index of Picamera ("picamera0")', 
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.75)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--record', help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                    action='store_true')
parser.add_argument('--display', help='display option ("monitor" or "pitft")', 
                    default="monitor")

args = parser.parse_args()


# Parse user inputs
model_path = args.model
img_source = args.source
min_thresh = float(args.thresh)
user_res = args.resolution
record = args.record
displayScreen = args.display


# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

if 'usb' in img_source:
    source_type = 'usb'
    usb_idx = int(img_source[3:])
else:
    print(f'Input {img_source} is invalid. Please try again.')
    sys.exit(0)

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Check if recording is valid and set up recording
if record:
    if source_type not in ['video','usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)
    
    # Set up recording
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

# Load or initialize image source
if source_type == 'image':
    imgs_list = [img_source]
elif source_type == 'folder':
    imgs_list = []
    filelist = glob.glob(img_source + '/*')
    for file in filelist:
        _, file_ext = os.path.splitext(file)
        if file_ext in img_ext_list:
            imgs_list.append(file)
elif source_type == 'video' or source_type == 'usb':

    if source_type == 'video': cap_arg = img_source
    elif source_type == 'usb': cap_arg = usb_idx
    cap = cv2.VideoCapture(cap_arg)

    # Set camera or video resolution if specified by user
    if user_res:
        ret = cap.set(3, resW)
        ret = cap.set(4, resH)

elif source_type == 'picamera':
    from picamera2 import Picamera2
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
    cap.start()

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# initialize window of object counts (for low pass filter if model keeps flashing in and out of detection)
object_count_history = deque(maxlen=2)

# inituialize window of object counts (for idle search)
object_count_history_idle = deque(maxlen=35000)
prevCount = 0

# Smoothed close-detection variables
smoothed_height = 0.0
smoothed_width = 0.0
alpha_height = 0.25
close_frames = 0
FRAME_REQUIRE_CLOSE = 4
HEIGHT_STOP_RATIO = 0.85  # ratio of frame height to count as "close"

# Begin inference loop
try:
    while True:
        t_start = time.perf_counter()

        # Access global distance variables from the background thread
        distance = DISTANCE
        bot_distance = BOT_DISTANCE
        
        # ====================================================================
        #                      PRIORITY 1: OBSTACLE AVOIDANCE
        # ====================================================================
        
        # if distance < STOP_DISTANCE_CM or bot_distance < STOP_DISTANCE_CM:
        #     move_back()
        #     continue
        
        # ====================================================================
        #                       END OBSTACLE AVOIDANCE
        # ====================================================================


        # Load frame from image source
        if source_type == 'image' or source_type == 'folder': # If source is image or image folder, load the image using its filename
            if img_count >= len(imgs_list):
                print('All images have been processed. Exiting program.')
                sys.exit(0)
            img_filename = imgs_list[img_count]
            frame = cv2.imread(img_filename)
            img_count = img_count + 1
        
        elif source_type == 'video': # If source is a video, load next frame from video file
            ret, frame = cap.read()
            if not ret:
                print('Reached end of the video file. Exiting program.')
                break
        
        elif source_type == 'usb': # If source is a USB camera, grab frame from camera
            ret, frame = cap.read()
            if (frame is None) or (not ret):
                print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')
                break

        elif source_type == 'picamera': # If source is a Picamera, grab frames using picamera interface
            frame = cap.capture_array()
            if (frame is None):
                print('Unable to read frames from the Picamera. This indicates the camera is disconnected or not working. Exiting program.')
                break

        # Resize frame to desired display resolution
        if resize == True:
            frame = cv2.resize(frame,(resW,resH))

        # Run inference on frame
        results = model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Initialize variable for basic object counting example
        object_count = 0
        movement_executed = False # Flag to track if we executed movement this frame

        # Go through each detection and get bbox coords, confidence, and class
        for i in range(len(detections)):

            # Get bounding box coordinates
            xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
            xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
            xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

            # Get bounding box class ID and name
            classidx = int(detections[i].cls.item())
            classname = labels[classidx]
            

            # Get bounding box confidence
            conf = detections[i].conf.item()

            print(classname, conf)

            # Draw box if confidence threshold is high enough
            if conf > min_thresh:

                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

                label = f'{classname}: {int(conf*100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text
                

                # Basic example: count the number of objects in the image
                object_count = object_count + 1

                # ============================================================
                #             PRIORITY 2: PERSON FOLLOWING (VISION)
                # ============================================================
                if classname == 'person':
                    spinMode = False
                    interuptEnable = False
                    prevCount = 0
                    
                    # make movement decisions based on object position
                    obj_center_x = (xmin + xmax) / 2
                    frame_center_x = frame.shape[1] / 2
                    offset_x = obj_center_x - frame_center_x
                    frame_height = frame.shape[0]
                    
                    # --------------------------------------------
                    # Better close detection using smoothed height
                    # --------------------------------------------
                    bboxHeight = ymax - ymin
                    
                    # exponential smoothing
                    smoothed_height = alpha_height * bboxHeight + (1 - alpha_height) * smoothed_height
                    smoothed_width = alpha_height * (xmax - xmin) + (1 - alpha_height) * smoothed_width
                    
                    # count consecutive close frames
                    # if smoothed_height > HEIGHT_STOP_RATIO * frame_height:
                    if smoothed_height > HEIGHT_STOP_RATIO * frame_height and smoothed_width > 0.6 * frame.shape[1]:
                        close_frames += 1
                    else:
                        close_frames = 0
                    
                    # if close for long enough ? stop
                    if close_frames >= FRAME_REQUIRE_CLOSE:
                        prevCount = 0

                        stop()
                        print(f'Close enough to {classname}, stopping.')
                        speak_espeak('got it!')
                        time.sleep(2)
                        # close_frames = 0
                        smoothed_height = 0.0
                        smoothed_width = 0.0
                        movement_executed = True
                        break
                        
                    else:
                        maxTurnDuty = 5
                        speed_x = offset_x/frame_center_x * maxTurnDuty # set x speed to the percentage of offset
                        print(f'offset_x: {offset_x}')
                        if abs(offset_x) < 20:
                            speed_x=0
                        speed_x = -1 * max(min(speed_x, maxTurnDuty), -maxTurnDuty)  # Clamp speed
                        
                        # Use HEIGHT to control forward speed
                        # dist_ratio = smoothed_height / frame_height     # 0.0 = far, 1.0 = very close
                        # Use area to control forward speed
                        bbox_area = (xmax - xmin) * (ymax - ymin)
                        frame_area = frame.shape[0] * frame.shape[1]
                        dist_ratio = bbox_area / frame_area  # 0.0 = far, 1.0 = very close
                        
                        # Want full speed when far, slow when close
                        speed_y = (1 - dist_ratio) * 100                 
                        speed_y = max(min(speed_y, 100), 0)  # Clamp speed to [0, 100]
                        left_speed = speed_y + speed_x 
                        right_speed = speed_y - speed_x
                        maxDuty = 50
                        max_speed = max(left_speed, right_speed)
                        left_speed /= max_speed
                        right_speed /= max_speed
                        left_speed *= maxDuty
                        right_speed *= maxDuty
                        left_speed = max(min(left_speed, maxDuty), -maxDuty)
                        right_speed = max(min(right_speed, maxDuty), -maxDuty)
                        
                        # Execute movement based on YOLO
                        move(left_speed, right_speed)
                        print(f'Tracking {classname}. Speeds L:{left_speed:.1f}, R:{right_speed:.1f}')
                        speak_espeak('wait up')
                        movement_executed = True # Mark that we followed a person
                        break # Only track the first person found
        
        
        # ============================================================
        #                  PRIORITY 3: IDLE SEARCH
        # ============================================================
        if not movement_executed:
            
            object_count_history.append(object_count)
            object_count_history_idle.append(object_count)
            prevCount += 1
            
            # If no object has been detected for 10 frames, stop and search

            if spinMode and prevCount >= 5:
                spinMode = False
                interuptEnable = False
                prevCount = 0
                print('No object detected for a while, spinning to search.')
                speak_espeak('where are you trash')

                # Spin right slowly
                move(50, -50) 
                time.sleep(0.4) # Spin for half a second
                stop()
                time.sleep(0.5)
                # reset idle count
                object_count_history_idle.append(1)
                prevCount = 0
            
            if prevCount >= 5:
                stop()
                time.sleep(0.2)
                interuptEnable = True
                prevCount = 0
                print('enabling interrupts')

        # Calculate and draw framerate (if using video, USB, or Picamera source)
        if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
            cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
        
        # Display detection results
        cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
        # Display sensor data
        cv2.putText(frame, f'Dist: {DISTANCE:.1f}cm', (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
        cv2.putText(frame, f'Bot Dist: {BOT_DISTANCE:.1f}cm', (10,80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)


        if displayScreen == 'monitor':
            cv2.imshow('YOLO detection results',frame) # Display image
        else:
            resized = cv2.resize(frame, (320,240), interpolation=cv2.INTER_NEAREST)
            frame16 = cv2.cvtColor(resized, cv2.COLOR_BGRA2BGR565)
            image_bytes = frame16.tobytes()
            f = open("/dev/fb0", 'wb')
            f.write(image_bytes)
            f.close()

        if record: recorder.write(frame)

        # If inferencing on individual images, wait for user keypress before moving to next image. Otherwise, wait 5ms before moving to next frame.
        if source_type == 'image' or source_type == 'folder':
            key = cv2.waitKey()
        elif source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
            key = cv2.waitKey(5)
        
        if key == ord('q') or key == ord('Q'): # Press 'q' to quit
            break
        elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
            cv2.waitKey()
        elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
            cv2.imwrite('capture.png',frame)
        
        # Calculate FPS for this frame
        t_stop = time.perf_counter()
        frame_rate_calc = float(1/(t_stop - t_start))

        # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
        if len(frame_rate_buffer) >= fps_avg_len:
            temp = frame_rate_buffer.pop(0)
            frame_rate_buffer.append(frame_rate_calc)
        else:
            frame_rate_buffer.append(frame_rate_calc)

        # Calculate average FPS for past frames
        avg_frame_rate = np.mean(frame_rate_buffer)
except KeyboardInterrupt:
    # Cleanup on exit
    stop()
    GPIO.cleanup() # Final cleanup
    pass

# Clean up
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if source_type == 'video' or source_type == 'usb':
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
cv2.destroyAllWindows()
GPIO.cleanup() # Final cleanup
# avoidance_thread.stop() # Stop the avoidance thread gracefully
# avoidance_thread.join() # Wait for the avoidance thread to finish
ultrasonic_thread.stop() # Stop the new thread gracefully
ultrasonic_thread.join() # Wait for the thread to finish
