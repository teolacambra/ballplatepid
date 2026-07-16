# arduino library importation
import serial 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 
serial_port = 'COM3'  # Replace with your Arduino's serial port
print(serial_port)
baud_rate = 9600  # Match the baud rate set in your Arduino sketch