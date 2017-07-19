import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/copy.csv"

gps_data = []

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
        x, y = requestHandler.math_convert(float(row[2]), float(row[3]))
        gps_data.append([x, y, float(row[9]), float(row[10])])

# The Kalman filter wants the GPS data in matrix form
gps_matrix = np.matrix(gps_data)

# Plot the GPS data
plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r', edgecolors="none")

# Run the Kalman filter
C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
output_matrix = kalman.kalman_no_loop(gps_matrix, C)

# Plot the Kalman output
plt.scatter(output_matrix[:,0], output_matrix[:,1], edgecolors="none")

# Show everything
plt.show()
