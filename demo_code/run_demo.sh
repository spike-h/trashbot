#!/bin/bash

sudo -E python light_test.py -c &
python detect_and_move_with_ultrasonic.py --model yolo11n_ncnn_model --source usb0 --resolution 480x320  --display pitft
sudo killall python