import csv
import numpy as np
import sys

import kalman
import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/gps_2017-07-11~~02-29-18-PM.csv"

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
rows = gps_matrix.shape[0]

# Find optimal values for C (translated from kalmanRunner in MATLAB)
min_error = sys.maxint
min_a = 0; min_b = 0; min_c = 0; min_d = 0
count = 0

for a in range(11):
    for b in range(11):
        for c in range(11):
            for d in range(11):
                
                C = np.matrix([[a/10., 0, 0, 0], [0, b/10., 0, 0], [0, 0, c/10., 0], [0, 0, 0, d/10.]])
                kalman_state = kalman.kalman_no_loop(gps_matrix, C)
                
                x_pos = gps_matrix[:,0]
                y_pos = gps_matrix[:,1]
                
                x_pos_kalman = kalman_state[:,0]
                y_pos_kalman = kalman_state[:,1]
                
                errorTotal = 0
                for i in range(rows):
                    error = np.sqrt((x_pos.item(i) - x_pos_kalman.item(i))**2 + (y_pos.item(i) - y_pos_kalman.item(i))**2)
                    errorTotal += error
                
                errorTotal /= rows
                print("Total error = ", errorTotal)
                
                if errorTotal < min_error:
                    min_error = errorTotal
                    min_a = a
                    min_b = b
                    min_c = c
                    min_d = d
                
                count += 1
                print(count)

print("Min error = ", min_error)
print("Min a = ", min_a)
print("Min b = ", min_b)
print("Min c = ", min_c)
print("Min d = ", min_d)

