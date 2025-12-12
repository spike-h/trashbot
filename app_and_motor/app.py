import threading
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify
from ultralytics import YOLO
import subprocess

MODEL_PATH = "yolo11n_ncnn_model" 
CAMERA_INDEX = 0       

app = Flask(__name__)

# ====================================
#         HARDWARE SETUP
# ====================================

# Motor Pins 
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
current_mode = "JOYSTICK"  
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
    # callibration to make motors go straight
    calibrated_right = right_speed * (1 - 12/75)
    
    set_motor(left_pwm, LEFT_INA, LEFT_INB, left_speed)
    set_motor(right_pwm, RIGHT_INA, RIGHT_INB, calibrated_right)

def stop_robot():
    set_motor(left_pwm, LEFT_INA, LEFT_INB, 0)
    set_motor(right_pwm, RIGHT_INA, RIGHT_INB, 0)

speech_process = None  # To track the running espeak process

def speak(text):
    global speech_process
    # If a process is already running and hasn't finished, SKIP this new speak command
    if speech_process is not None and speech_process.poll() is None:
        return 
    
    # Otherwise, start a new one
    try:
        speech_process = subprocess.Popen(["espeak", text])
    except Exception as e:
        print(f"Audio error: {e}")


# ====================================
#       SOUND DETECTION SETUP
# ====================================

# Define the GPIO pin connected to the sound sensor's D0 pin
SOUND_SENSOR_PIN = 4 
spinMode = False
interuptEnable = False

GPIO.setup(SOUND_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

def sound_detected_callback(channel):

    global interuptEnable
    if interuptEnable:
        current_time = time.strftime("%H:%M:%S")
        print(f"\n--- {current_time} --- ?? Sound Detected!")
        speak('where are you trash')
        change_lights('theater', '0,0,255')
        # Spin right slowly
        move_robot(50, -50) 
        time.sleep(0.4) # Spin for half a second
        stop_robot()
        time.sleep(0.5)
        interuptEnable = False
 
GPIO.add_event_detect(
    SOUND_SENSOR_PIN, 
    GPIO.RISING, 
    callback=sound_detected_callback, 
    bouncetime=200
)


# ====================================
#     END SOUND DETECTION SETUP
# ===================================

timeoutLED = 5
ledStart = -10
LED_process= None

def change_lights(mode='off', color="255,255,255"):
    """
    Calls the light_test.py script with sudo privileges.
    mode: 'wipe', 'rainbow', 'theater', 'off'
    color: string 'R,G,B'
    """
    try:
        
        cmd = ["sudo", "-E", "python3", "light_test.py", "--mode", mode, "--color", color]
        
        # subprocess.Popen allows the script to run without blocking your app completely,
        # or use subprocess.run() if you want to wait for the animation to finish.
        
        global LED_process
        # If a process is already running and hasn't finished, SKIP this new speak command
        if LED_process is not None and LED_process.poll() is None:
            return 
    
        # Otherwise, start a new one
        LED_process = subprocess.Popen(cmd)
        #global ledStart
        #if time.time() - ledStart > 5:
        #    subprocess.run(cmd)
        #    ledStart = time.time()
        
    except Exception as e:
        print(f"Error controlling lights: {e}")


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

STOP_DISTANCE_CM = 10
def ultrasonic_loop():
    global distance_main, distance_bot
    while running:
        distance_main = get_distance(TRIG_PIN, ECHO_PIN)
        distance_bot = get_distance(BOT_TRIG_PIN, BOT_ECHO_PIN)
        time.sleep(0.1)

# ====================================
#      AUTONOMOUS LOGIC THREAD
# ====================================
displayScreen = 'pitft'  # 'monitor' or 'pitft'
smoothed_height = 0.0
smoothed_width = 0.0
# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0
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
    prevCount = 0
    global avg_frame_rate, interuptEnable

    # --- OPTIMIZATION 1: Open the screen file ONCE ---
    fb_file = None
    if displayScreen == 'pitft':
        try:
            fb_file = open("/dev/fb0", 'wb')
        except Exception as e:
            print(f"Could not open screen: {e}")
    
    while running:
        if current_mode != "AUTONOMOUS":
            time.sleep(0.2)
            continue

        t_start = time.perf_counter()
        # 1. OBSTACLE AVOIDANCE (Priority High)
        if distance_main < STOP_DISTANCE_CM or distance_bot < STOP_DISTANCE_CM:
            print(f"Obstacle detected! Prioritizing avoidance. top Dist {distance_main}, bot dist {distance_bot}")
            speak("ahhhh too close")
            stop_robot()
            move_robot(-60, -60)
            change_lights('wipe', '255,0,0')
            time.sleep(1.0)
            stop_robot()
            continue

        # 2. VISION TRACKING
        ret, frame = cap.read()
        if not ret: continue
        
        # Resize for speed
        frame = cv2.resize(frame, (480, 320)) 
        results = model(frame, verbose=False)
        detections = results[0].boxes
        object_count = 0
        
        person_found = False
        movement_executed = False
        
        for i in range(len(detections)):
            # Extract Data
            cls_id = int(detections[i].cls.item())
            class_name = model.names[cls_id]
            conf = detections[i].conf.item()
            
            if class_name == 'person' and conf > 0.7:
                person_found = True
                prevCount = 0
                xyxy = detections[i].xyxy.cpu().numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy
                
                # Logic from your file: Calculate offsets
                obj_center_x = (xmin + xmax) / 2
                frame_center_x = frame.shape[1] / 2 # Half of width
                print(f"FRAME CENTER X IS {frame_center_x} SHOULD BE 240")
                frame_height = frame.shape[0]
                alpha_height = 0.25
                global smoothed_height, smoothed_width
                offset_x = obj_center_x - frame_center_x
                
                # Turn Logic
                turn_speed = (offset_x / frame_center_x) * 40 # Scale turn

                # --------------------------------------------
                # Better close detection using smoothed height
                # --------------------------------------------
                bboxHeight = ymax - ymin
                
                # exponential smoothing for low pass filter of height and width change
                smoothed_height = alpha_height * bboxHeight + (1 - alpha_height) * smoothed_height
                smoothed_width = alpha_height * (xmax - xmin) + (1 - alpha_height) * smoothed_width
                
                # count consecutive close frames
                HEIGHT_STOP_RATIO = 0.8
                if smoothed_height > HEIGHT_STOP_RATIO * frame_height and smoothed_width > 0.6 * frame.shape[1]:
                    close_frames += 1
                else:
                    close_frames = 0
                
                # if close for long enough ? stop
                if close_frames >= FRAME_REQUIRE_CLOSE:
                    prevCount = 0

                    stop_robot()
                    print(f'Close enough to {class_name}, stopping.')
                    speak('got it!')
                    change_lights('wipe', "255,0,0")
                    time.sleep(2)
                    # close_frames = 0
                    smoothed_height = 0.0
                    smoothed_width = 0.0
                    movement_executed = True
                    break
                    
                else:
                    maxTurnDuty = 5
                    print(f'offset_x: {offset_x}')
                    if offset_x < 0:
                        signX= -1
                    else: 
                        signX = 1
                    tolerance = 40

                    speed_x = (offset_x-signX*tolerance)/frame_center_x * maxTurnDuty # set x speed to the percentage of offset


                    if abs(offset_x) < tolerance:
                        speed_x=0
                    speed_x = -1 * max(min(speed_x, maxTurnDuty), -maxTurnDuty)  # Clamp speed
                    
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
                    
                    move_robot(left_speed, right_speed)
                    print(f'Tracking {class_name}. Speeds L:{left_speed:.1f}, R:{right_speed:.1f}')
                    speak('wait up')
                    change_lights('theater', '0,255,0')
                    movement_executed = True 
                
                break # Only track the first person found

        if not person_found:
            stop_robot()
        # ============================================================
        #                  PRIORITY 3: IDLE SEARCH
        # ============================================================
        if not movement_executed:

            prevCount += 1
            
            # only start to look for claps after 5 frames of not seeing anyone
            # or else motors will trigger sounds
            if prevCount >= 5 and not interuptEnable:
                stop_robot()
                time.sleep(0.2)
                interuptEnable = True
                change_lights('wipe', '255,255,0')
                prevCount = 0
                print('enabling interrupts')


        # draw to screen
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
        cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
        # Display sensor data
        cv2.putText(frame, f'Dist: {distance_main:.1f}cm', (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
        cv2.putText(frame, f'Bot Dist: {distance_bot:.1f}cm', (10,80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)


        # display to monitor or ssh screen
        if displayScreen == 'monitor':
            cv2.imshow('YOLO', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # display to pitft
        elif fb_file:
            resized = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_NEAREST)
            frame16 = cv2.cvtColor(resized, cv2.COLOR_BGRA2BGR565)
            
            # Reset file pointer to start instead of closing/reopening
            fb_file.seek(0)
            fb_file.write(frame16.tobytes())
            fb_file.flush() # Ensure data is sent immediately

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

    cap.release()
    if fb_file:
        fb_file.close()

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
        change_lights('wipe', "255,255,255")
    else:
        speak("Joystick Mode")
        change_lights('rainbow')
        
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
    turn = -x * 80 # Reduced turn sensitivity slightly

    left = speed + turn
    right = speed - turn
    
    # Clamp
    left = max(min(left, 80), -80)
    right = max(min(right, 80), -80)

    move_robot(left, right)
    return jsonify({"status": "success", "l": left, "r": right})

if __name__ == '__main__':
    # Start Sensors in Background
    t_ultra = threading.Thread(target=ultrasonic_loop)
    t_ultra.daemon = True
    t_ultra.start()

    # Start auto mode in Background
    t_auto = threading.Thread(target=autonomous_loop)
    t_auto.daemon = True
    t_auto.start()

    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        running = False
        change_lights()
        GPIO.cleanup()