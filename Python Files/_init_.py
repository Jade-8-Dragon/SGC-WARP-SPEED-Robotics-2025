"""
SGC WARP SPEED Robotics 2025 - Python Package
"""

__version__ = "3/25/2006"
__author__ = "WARP SPEED Robotics Team"

# Import main modules here as needed
# Example:
# from . import robot
import serial
import time 

arduino = serial.Serial('/dev/ttyUSB0', 115200)