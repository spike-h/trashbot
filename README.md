# trashbot

Trashbot, AKA Binny, is a trashcan that can either be remotely controlled or autonomously follow you around. Equipped with a speaker, microphone, camera, LEDs, and a locally hosted network, this piece of garbage is made to help you out.

File Structure:

    **Demo Code:**

    -----> run_demo.sh
            - bash script to run the LED and movement code at once. This script is run via ./run_demo.sh

    -----> light_test.py
            - Python script to turn on the LEDs and loop through an animation once.

    -----> detect_and_move_with_ultrasonic.py
            - Python script to handle the GPIO setup, connecting to bluetooth speaker, image acquiring and processing, motor control, LCD Display, ultrasonic sensors, microphone, (literally everything that isnt an LED).


    **app_and_motor**
    - used in video

    -----> static
           ------> nipplejs.min.js
                   - Joystick javascript libray
    -----> templates
           ------> index.html
                   - simple web page to house joystick and toggle button
    -----> yolo11n_ncnn_model
           - yolo model
    -----> app.py
           - Flask website server and entire logic for controlling motors.
    -----> light_test.py
           - Python script to control LEDs, must be run as sudo
    -----> RUN BY DOING python app.py

**website**
----> website with report
