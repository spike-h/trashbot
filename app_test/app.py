from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO

app = Flask(__name__)

# --- CONFIGURATION ---
# Motor Pins (BCM numbering)
# Motor A
IN1 = 17
IN2 = 18
# Motor B
IN3 = 22
IN4 = 23
# PWM Pins for speed control (Optional, using simple logic here)
ENA = 24
ENB = 25

# --- SETUP GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# PWM Setup (Frequency = 1000Hz)
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

# Global State
current_mode = "JOYSTICK" # Options: JOYSTICK, DISCRETE

def set_motor(motor, speed):
    """
    Controls motor direction and speed.
    speed: -100 (full reverse) to 100 (full forward)
    motor: 'A' or 'B'
    """
    in1, in2, pwm = (IN1, IN2, pwm_a) if motor == 'A' else (IN3, IN4, pwm_b)
    
    pwm.ChangeDutyCycle(abs(speed))
    
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif speed < 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/toggle_mode', methods=['POST'])
def toggle_mode():
    global current_mode
    data = request.json
    current_mode = data.get('mode', 'JOYSTICK')
    print(f"Mode switched to: {current_mode}")
    # Stop motors when switching modes for safety
    set_motor('A', 0)
    set_motor('B', 0)
    return jsonify({"status": "success", "mode": current_mode})

@app.route('/joystick_data', methods=['POST'])
def joystick_data():
    if current_mode != "JOYSTICK":
        return jsonify({"status": "ignored", "reason": "wrong_mode"})

    data = request.json
    # x and y usually come in range -1.0 to 1.0 logic from frontend
    x = data.get('x', 0)
    y = data.get('y', 0)

    # Simple Tank Drive Mixing Logic
    # Convert x/y float to -100 to 100 speed
    speed = y * 100
    turn = x * 100

    left_speed = speed + turn
    right_speed = speed - turn

    # Clamp values to -100 to 100
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    set_motor('A', left_speed)
    set_motor('B', right_speed)

    return jsonify({"status": "success", "l": left_speed, "r": right_speed})

@app.route('/discrete_control', methods=['POST'])
def discrete_control():
    if current_mode != "DISCRETE":
        return jsonify({"status": "ignored", "reason": "wrong_mode"})
        
    data = request.json
    action = data.get('action')
    
    # Simple discrete movements
    if action == 'up':
        set_motor('A', 50)
        set_motor('B', 50)
    elif action == 'down':
        set_motor('A', -50)
        set_motor('B', -50)
    elif action == 'left':
        set_motor('A', -50)
        set_motor('B', 50)
    elif action == 'right':
        set_motor('A', 50)
        set_motor('B', -50)
    else: # Stop
        set_motor('A', 0)
        set_motor('B', 0)
        
    return jsonify({"status": "success"})

if __name__ == '__main__':
    # Host 0.0.0.0 makes it accessible on the network
    app.run(host='0.0.0.0', port=80, debug=False)