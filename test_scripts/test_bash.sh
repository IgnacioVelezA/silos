#!/bin/bash

# Run script_A
python3 script_A.py

# Wait for script_A to finish and the flag file to appear
while [ ! -f "script_A_finished.flag" ]; do
    sleep 1  # Adjust the sleep duration as needed
done

# Run script_B once the flag file appears
python3 script_B.py
