#!/bin/bash

# Run script_A
python3 ./silos/src/scripts/background_scripts/temperature.py

iteration=19
title="medicion_prueba_basurero_"
ard_usb="/dev/ttyACM0"
r0=3
rmax=30
radar_usb="/dev/ttyUSB0"

while :; do
    filename="$title$iteration"
    python3 ./silos/src/scripts/silos_measurement_hex.py $filename $ard_usb $r0 $rmax $radar_usb
    # Wait for script_A to finish and the flag file to appear
    while [ ! -f "script_A_finished.flag" ]; do
        sleep 1  # Adjust the sleep duration as needed
    done
    iteration=$((iteration + 1))
done

