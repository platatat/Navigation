import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/gps_2017-08-02~~12-46-23-PM.csv"
KALMAN_FILE = "/Users/joshuasones/Desktop/kalman_pub_2017-08-02~~12-46-23-PM.csv"

gps_data = []
kalman_data = []

with open(GPS_FILE) as gps_file:
    gps_reader = csv.reader(gps_file, delimiter=",")
    header_row = True
    for row in gps_reader:
        if header_row:

            # Skip the first row, which contains the headers
            header_row = False
            continue

        # field0 is lat, field1 is long, field7 is yaw in degrees,
        # field8 is speed from the gps (meters per second)
        # field10 is timestep (miliseconds)
        x, y = requestHandler.math_convert(float(row[2]), float(row[3]))
        gps_data.append([x, y, float(row[9])*np.pi/180, float(row[10]), float(row[12])])
        
with open(KALMAN_FILE) as kalman_file:
    kalman_reader = csv.reader(kalman_file, delimiter=",")
    header_row = True
    for row in kalman_reader:
        if header_row:

            # Skip the first row, which contains the headers
            header_row = False
            continue

        # field5 is lat converted to x, field6 is long converted to y - cartesian
        kalman_data.append([float(row[5]), float(row[6])])

# The Kalman filter wants the GPS data in matrix form
gps_matrix = np.matrix(gps_data)
kalman_matrix = np.matrix(kalman_data)

# Plot the GPS data
plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r', edgecolors="none")

# Plot the Kalman output
#plt.scatter(kalman_matrix[:,0], kalman_matrix[:,1], edgecolors="none")

# Show everything
plt.show()
