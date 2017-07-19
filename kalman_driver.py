import csv
import matplotlib
import numpy as np
import gps_assisted_simulator_node 

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

GPS_FILE = "/home/daniel/Desktop/gps_2017-07-06~~03-28-40-PM.csv"

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
plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r')

# Run the Kalman filter
output_matrix = kalman.kalman_no_loop(gps_matrix)

# Plot the Kalman output
plt.scatter(output_matrix[:,0], output_matrix[:,1])

# Show everything
plt.show()
