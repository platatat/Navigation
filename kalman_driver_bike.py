"""Script that gets location data from gps csv and speed/yaw from bike csv,
as opposed to kalman_driver.py which only gets all data from gps"""

import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/gps_2017-07-18~~02-36-30-PM-copy.csv"

gps_data = []

with open(GPS_FILE) as gps_file:
    gps_reader = csv.reader(gps_file, delimiter=",")
    header_row = True
    for row in gps_reader:
        if header_row:

            # Skip the first row, which contains the headers
            header_row = False
            continue

        # field0 is lat, field1 is long, field7 is yaw
        x, y = requestHandler.math_convert(float(row[2]), float(row[3]))
        gps_data.append([x, y, float(row[9])])

BIKE_STATE_FILE = "/Users/joshuasones/Desktop/bike_2017-07-18~~02-36-30-PM-copy.csv"

bike_state_data = []

with open(BIKE_STATE_FILE) as bike_state_file:
    bike_state_reader = csv.reader(bike_state_file, delimiter=",")
    header_row = True
    for row in bike_state_reader:
        if header_row:
            
            #Skip the first row, which contains the headers
            header_row = False
            continue
        
        #field9 is yaw
        bike_state_data.append([180+float(row[11])*180/np.pi])

# The Kalman filter wants the data in combined matrix form
gps_matrix = np.matrix(gps_data)
bike_state_matrix = np.matrix(bike_state_data)
combined_matrix = np.insert(gps_matrix, 2, bike_state_matrix.flatten(), axis=1)
#print combined_matrix.shape

# Plot the GPS data
plt.scatter(combined_matrix[:,0], combined_matrix[:,1], c='r', edgecolors="none")

# Run the Kalman filter
C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
output_matrix = kalman.kalman_no_loop(combined_matrix, C)

# Plot the Kalman output
plt.scatter(output_matrix[:,0], output_matrix[:,1], edgecolors="none")

# Show everything
plt.show()
