import threading
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify
from ultralytics import YOLO
import subprocess

# --- GLOBAL SETTINGS ---
MODEL_PATH = "../yolo11n_ncnn_model.pt"  # <--- CHANGE THIS to your model file
CAMERA_INDEX = 0        # 0 for USB, 'picamera' for PiCamera (requires extra setup)

app = Flask(__name__)

# ====================================
#         HARDWARE SETUP
# ====================================

# Motor Pins (From your uploaded file)
LEFT_INA = 5
LEFT_INB = 6
LEFT_PWM = 26
RIGHT_INA = 17
RIGHT_INB = 20
RIGHT_PWM = 16

# Ultrasonic Pins
TRIG_PIN = 13
ECHO_PIN = 19
BOT_TRIG_PIN = 27
BOT_ECHO_PIN = 22

# Sound Sensor
SOUND_SENSOR_PIN = 4

# GPIO Init
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Setup
GPIO.setup([LEFT_INA, LEFT_INB, LEFT_PWM, RIGHT_INA, RIGHT_INB, RIGHT_PWM], GPIO.OUT)
left_pwm = GPIO.PWM(LEFT_PWM, 50)
right_pwm = GPIO.PWM(RIGHT_PWM, 50)
left_pwm.start(0)
right_pwm.start(0)

# Sensor Setup
GPIO.setup([TRIG_PIN, BOT_TRIG_PIN], GPIO.OUT)
GPIO.setup([ECHO_PIN, BOT_ECHO_PIN], GPIO.IN)
GPIO.output(TRIG_PIN, False)
GPIO.output(BOT_TRIG_PIN, False)

# Global State Variables
current_mode = "JOYSTICK"  # Starts in Joystick mode
distance_main = 400
distance_bot = 400
running = True

# ====================================
#         MOTOR FUNCTIONS
# ====================================

def set_motor(pwm, ina, inb, speed):
    """
    Controls a single motor. Speed: -100 to 100
    """
    pwm.ChangeDutyCycle(abs(speed))
    if speed > 0:
        GPIO.output(ina, GPIO.HIGH)
        GPIO.output(inb, GPIO.LOW)
    elif speed < 0:
        GPIO.output(ina, GPIO.LOW)
        GPIO.output(inb, GPIO.HIGH)
    else:
        GPIO.output(ina, GPIO.LOW)
        GPIO.output(inb, GPIO.LOW)

def move_robot(left_speed, right_speed):
    # Apply your specific calibration (from uploaded file)
    # Right motor was slightly stronger in your code: right_speed * (1-12/75)
    calibrated_right = right_speed * (1 - 12/75)
    
    set_motor(left_pwm, LEFT_INA, LEFT_INB, left_speed)
    set_motor(right_pwm, RIGHT_INA, RIGHT_INB, calibrated_right)

def stop_robot():
    set_motor(left_pwm, LEFT_INA, LEFT_INB, 0)
    set_motor(right_pwm, RIGHT_INA, RIGHT_INB, 0)

def speak(text):
    # Non-blocking speech
    subprocess.Popen(["espeak", text])

# ====================================
#      ULTRASONIC THREAD
# ====================================
def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    start_time = time.time()
    stop_time = time.time()
    
    timeout = time.time() + 0.04
    while GPIO.input(echo) == 0 and time.time() < timeout:
        start_time = time.time()
    while GPIO.input(echo) == 1 and time.time() < timeout:
        stop_time = time.time()
        
    dist = ((stop_time - start_time) * 34300) / 2
    return min(dist, 400)

def ultrasonic_loop():
    global distance_main, distance_bot
    while running:
        distance_main = get_distance(TRIG_PIN, ECHO_PIN)
        distance_bot = get_distance(BOT_TRIG_PIN, BOT_ECHO_PIN)
        time.sleep(0.1)

# ====================================
#      AUTONOMOUS LOGIC THREAD
# ====================================
def autonomous_loop():
    print("Loading YOLO model...")
    try:
        model = YOLO(MODEL_PATH) 
        print("Model Loaded!")
    except Exception as e:
        print(f"Error loading model: {e}. Make sure {MODEL_PATH} is in the folder.")
        return

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Camera not found!")
        return

    # Tracking Variables
    close_frames = 0
    FRAME_REQUIRE_CLOSE = 4
    
    while running:
        if current_mode != "AUTONOMOUS":
            time.sleep(0.2)
            continue

        # 1. OBSTACLE AVOIDANCE (Priority High)
        if distance_main < 15 or distance_bot < 15:
            print("Obstacle! Backing up.")
            speak("too close")
            stop_robot()
            move_robot(-50, -50)
            time.sleep(1.0)
            stop_robot()
            continue

        # 2. VISION TRACKING
        ret, frame = cap.read()
        if not ret: continue
        
        # Resize for speed
        frame = cv2.resize(frame, (640, 480)) 
        results = model(frame, verbose=False)
        detections = results[0].boxes
        
        person_found = False
        
        for i in range(len(detections)):
            # Extract Data
            cls_id = int(detections[i].cls.item())
            class_name = model.names[cls_id]
            conf = detections[i].conf.item()
            
            if class_name == 'person' and conf > 0.5:
                person_found = True
                xyxy = detections[i].xyxy.cpu().numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy
                
                # Logic from your file: Calculate offsets
                obj_center_x = (xmin + xmax) / 2
                frame_center_x = 320 # Half of 640
                offset_x = obj_center_x - frame_center_x
                
                # Turn Logic
                turn_speed = (offset_x / frame_center_x) * 40 # Scale turn
                
                # Forward Logic (Simple Area based)
                height = ymax - ymin
                if height > 400: # Too close
                    stop_robot()
                    speak("got it")
                    time.sleep(2)
                else:
                    forward_speed = 50
                    move_robot(forward_speed + turn_speed, forward_speed - turn_speed)
                
                break # Only track one person

        if not person_found:
            stop_robot()
            # Optional: Add your "Idle Search / Spin" logic here

    cap.release()

# ====================================
#          FLASK ROUTES
# ====================================

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/toggle_mode', methods=['POST'])
def toggle_mode():
    global current_mode
    data = request.json
    new_mode = data.get('mode')
    
    stop_robot() # Stop before switching
    current_mode = new_mode
    
    if current_mode == 'AUTONOMOUS':
        speak("Autonomous Mode")
    else:
        speak("Joystick Mode")
        
    print(f"Mode switched to: {current_mode}")
    return jsonify({"status": "success", "mode": current_mode})

@app.route('/joystick_data', methods=['POST'])
def joystick_data():
    if current_mode != "JOYSTICK":
        return jsonify({"status": "ignored"})

    data = request.json
    x = data.get('x', 0)
    y = data.get('y', 0)

    # Convert Joystick Float (-1.0 to 1.0) to PWM (-100 to 100)
    speed = y * 100
    turn = x * 80 # Reduced turn sensitivity slightly

    left = speed + turn
    right = speed - turn
    
    # Clamp
    left = max(min(left, 100), -100)
    right = max(min(right, 100), -100)

    move_robot(left, right)
    return jsonify({"status": "success", "l": left, "r": right})

if __name__ == '__main__':
    # Start Sensors in Background
    t_ultra = threading.Thread(target=ultrasonic_loop)
    t_ultra.daemon = True
    t_ultra.start()

    # Start Autonomous AI in Background
    t_auto = threading.Thread(target=autonomous_loop)
    t_auto.daemon = True
    t_auto.start()

    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        running = False
        GPIO.cleanup()