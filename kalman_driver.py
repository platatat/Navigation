"""
Used to plot raw GPS data vs. kalman-filtered data. In order to use this, you must
download the correct csv files (see testing instructions), and change the relevant filepaths.

At the bottom of the file, uncomment the files and function you want to use for plotting:

Use gps_only_retro() to plot kalman filter data made retroactively using only the gps.
You must uncomment GPS_FILE for this.

Use gps_imu_retro() to plot kalman filter data made retroactively using gps and imu data.
You must uncomment GPS_FILE and BIKE_STATE for this.

Use real_time() to plot kalman filter data that was made in real time during a test.
You must uncomment GPS_FILE and KALMAN_FILE for this.
"""

import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

def gps_only_retro(gps_file):
  
  gps_data = []
  with open(gps_file) as gps_file:
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
        
  # The Kalman filter wants the GPS data in matrix form
  gps_matrix = np.matrix(gps_data)
  
  # Plot the GPS data
  plt.scatter([gps_matrix[:,0]], [gps_matrix[:,1]], c='r', edgecolors="none")
  
  # Run the Kalman filter
  output_matrix = kalman.kalman_retro(gps_matrix)
  
  # Plot the Kalman output
  plt.scatter([output_matrix[:,0]], [output_matrix[:,1]], edgecolors="none")
  
  # Show everything
  plt.show()

def gps_imu_retro(gps_file, bike_state_file):
  
  count = 0
  orig_lat = 0
  orig_long
  gps_data = []
  with open(gps_file) as gps_file:
      gps_reader = csv.reader(gps_file, delimiter=",")
      header_row = True
      for row in gps_reader:
          if header_row:
  
              # Skip the first row, which contains the headers
              header_row = False
              continue
  
          # field0 is lat, field1 is long
          # field8 is speed from the gps (meters per second)
          # field10 is timestep (miliseconds)
           #Uncomment if not using relative
          # x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), float(row[2]), float(row[3]))

          #Uncomment up 'else' if using relative
          if count == 0:
            orig_lat = float(row[2])
            orig_long = float(row[3])
            x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat , orig_long )
            count = 1
          else:
            x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat, orig_long)
          gps_data.append([x, y, float(row[10]), float(row[12])])
  
          
  bike_state_data = []
  with open(bike_state_file) as bike_state_file:
    bike_state_reader = csv.reader(bike_state_file, delimiter=",")
    header_row = True
    for row in bike_state_reader:
        if header_row:
            
            #Skip the first row, which contains the headers
            header_row = False
            continue
        
        #field9 is yaw in radians
        bike_state_data.append([180+float(row[11])])
  
  # The Kalman filter wants the data in combined matrix form
  gps_matrix = np.matrix(gps_data)
  bike_state_matrix = np.matrix(bike_state_data)
  combined_matrix = np.insert(gps_matrix, 2, bike_state_matrix.flatten(), axis=1)
  
  # Plot the GPS data
  plt.scatter([combined_matrix[:,0]], [combined_matrix[:,1]], c='r', edgecolors="none")
  
  # Run the Kalman filter
  output_matrix = kalman.kalman_retro(combined_matrix)
  
  # Plot the Kalman output
  plt.scatter([output_matrix[:,0]], [output_matrix[:,1]], edgecolors="none")
  
  # Show everything
  plt.show()
  
def real_time(gps_file, kalman_file):
  
  gps_data = []
  count = 0
  orig_lat = 0
  orig_long = 0
  with open(gps_file) as gps_file:
      gps_reader = csv.reader(gps_file, delimiter=",")
      header_row = True
      for row in gps_reader:
          if header_row:
  
              # Skip the first row, which contains the headers
              header_row = False
              continue
  
          # field0 is lat, field1 is long
          # field8 is speed from the gps (meters per second)
          # field10 is timestep (miliseconds)
          #Uncomment if not using relative
          # x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), float(row[2]), float(row[3]))

          #Uncomment up 'else' if using relative
          if count == 0:
            orig_lat = float(row[2])
            orig_long = float(row[3])
            x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat , orig_long )
            count = 1
          else:
            x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat, orig_long)
          gps_data.append([x, y, float(row[10]), float(row[12])])
  
  kalman_data = []    
  with open(kalman_file) as kalman_file:
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
  plt.scatter([gps_matrix[:,0]], [gps_matrix[:,1]], c='r', edgecolors="none")
  
  # Plot the Kalman output
  plt.scatter([kalman_matrix[:,0]], [kalman_matrix[:,1]], edgecolors="none")
  
  #PLOT WAYPOINTS ONCE WE HAVE THE CHANCE
  waypoints = [(0.0, 0.0), (0.0, 10.0), (0.0, 20.0), (0.0, 30.0)]
  
  plt.scatter([i[0] for i in waypoints], [i[1] for i in waypoints], c='g')
  
  # Show everything
  plt.show()
  
if __name__ == '__main__':
  GPS_FILE = "/Users/joshua/Desktop/gps_2017-11-07~~07-27-22-PM.csv"
  #BIKE_STATE = "/Users/joshua/Desktop/kalman_pub_2017-11-07~~07-27-22-PM.csv"
  KALMAN_FILE = "/Users/joshua/Desktop/kalman_pub_2017-11-07~~07-27-22-PM.csv"
  
  gps_only_retro(GPS_FILE)
  #gps_imu_retro(GPS_FILE, BIKE_STATE)
  #real_time(GPS_FILE, KALMAN_FILE)
