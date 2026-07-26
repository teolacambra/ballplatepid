import serial 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 

testx = np.random.uniform(-2, 2, size=100) # Generate random x values between -2 and 2
testy = np.random.uniform(-2, 2, size=100) # Generate random y values between -2 and 2
location = np.column_stack((testx, testy)) # Combine x and y values into a 2D array
print(location) # Print the location array