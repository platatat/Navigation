import csv
import numpy as np
import sys

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
rows = combined_matrix.shape[0]

# Find optimal values for C (translated from kalmanRunner in MATLAB)
min_error = sys.maxint
min_a = 0; min_b = 0; min_c = 0; min_d = 0
count = 0

for a in range(11):
    for b in range(11):
        for c in range(11):
            for d in range(11):
                
                C = np.matrix([[a/10., 0, 0, 0], [0, b/10., 0, 0], [0, 0, c/10., 0], [0, 0, 0, d/10.]])
                kalman_state = kalman.kalman_no_loop(combined_matrix, C)
                
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
                    min_a = a/10.
                    min_b = b/10.
                    min_c = c/10.
                    min_d = d/10.
                
                count += 1
                print(count)

print("Min error = ", min_error)
print("Min a = ", min_a)
print("Min b = ", min_b)
print("Min c = ", min_c)
print("Min d = ", min_d)

