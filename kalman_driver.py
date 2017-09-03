"""
Used to plot raw GPS data vs. kalman-filtered data. In order to use this, you must
download the correct csv files (see testing instructions), and change the relevant filepaths
(starting at line 28).

At the bottom of the file, uncomment the function you want to use for plotting:

Use gps_only_retro() to plot kalman filter data made retroactively using only the gps.
You need the correct GPS_FILE for this.

Use gps_imu_retro() to plot kalman filter data made retroactively using gps and imu data.
You need the correct GPS_FILE and BIKE_STATE for this.

Use real_time() to plot kalman filter data that was made in real time during a test.
You need the correct GPS_FILE and KALMAN_FILE for this.
"""

import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/School/Bike Team/gps_2017-07-18~~02-36-30-PM-copy.csv"
BIKE_STATE_FILE = "/Users/joshuasones/Desktop/bike_2017-07-18~~02-36-30-PM-copy.csv"
KALMAN_FILE = "/Users/joshuasones/Desktop/School/Bike Team/kalman_pub_2017-08-02~~12-29-37-PM.csv"

gps_psyt_data = []
gps_pst_data = []
bikestate_y_data = []
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
        gps_psyt_data.append([x, y, float(row[9])*np.pi/180, float(row[10]), float(row[12])])
        gps_pst_data.append([x, y, float(row[10]), float(row[12])])

with open(BIKE_STATE_FILE) as bike_state_file:
    bike_state_reader = csv.reader(bike_state_file, delimiter=",")
    header_row = True
    for row in bike_state_reader:
        if header_row:
            
            #Skip the first row, which contains the headers
            header_row = False
            continue
        
        #field9 is yaw in radians
        bikestate_y_data.append([180+float(row[11])])
        
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

def gps_only_retro(gps_data):
  # The Kalman filter wants the GPS data in matrix form
  gps_matrix = np.matrix(gps_data)
  
  # Plot the GPS data
  plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r', edgecolors="none")
  
  # Run the Kalman filter
  output_matrix = kalman.kalman_retro(gps_matrix)
  
  # Plot the Kalman output
  plt.scatter(output_matrix[:,0], output_matrix[:,1], edgecolors="none")
  
  # Show everything
  plt.show()

def gps_imu_retro(gps_data, imu_data):
  
  # The Kalman filter wants the data in combined matrix form
  gps_matrix = np.matrix(gps_data)
  bike_state_matrix = np.matrix(imu_data)
  combined_matrix = np.insert(gps_matrix, 2, bike_state_matrix.flatten(), axis=1)
  
  # Plot the GPS data
  plt.scatter(combined_matrix[:,0], combined_matrix[:,1], c='r', edgecolors="none")
  
  # Run the Kalman filter
  output_matrix = kalman.kalman_retro(combined_matrix)
  
  # Plot the Kalman output
  plt.scatter(output_matrix[:,0], output_matrix[:,1], edgecolors="none")
  
  # Show everything
  plt.show()
  
def real_time(gps_data, kalman_data):
  # The Kalman filter wants the GPS data in matrix form
  gps_matrix = np.matrix(gps_data)
  kalman_matrix = np.matrix(kalman_data)
  
  # Plot the GPS data
  plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r', edgecolors="none")
  
  # Plot the Kalman output
  #plt.scatter(kalman_matrix[:,0], kalman_matrix[:,1], edgecolors="none")
  
  # Show everything
  plt.show()
  
if __name__ == '__main__':
  #gps_only_retro(gps_psyt_data)
  gps_imu_retro(gps_pst_data, bikestate_y_data)
  #real_time(gps_psyt_data, kalman_data)
