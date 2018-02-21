import csv
import numpy as np
import requestHandler
from collections import namedtuple


GPSData = namedtuple('GPSData', ['x', 'y', 'yaw', 'speed', 'timestamp'])


class SensorData:
    def __init__(self, gps_filename=None, bike_state_filename=None):
        if gps_filename is not None:
            self.gps = self.load_gps_state(gps_filename)

        if bike_state_filename is not None:
            self.bike = self.load_bike_state(bike_state_filename)


    def load_gps_state(self, gps_filename):
        x_data = []
        y_data = []
        yaw_data = []
        speed_data = []
        timestamp_data = []

        with open(gps_filename) as gps_file:
            csv_reader = csv.reader(gps_file, delimiter=',')

            time_offset = None

            # skip header row
            for row in list(csv_reader)[1:]:
                x, y = requestHandler.math_convert(float(row[2]), float(row[3]))
                yaw = float(row[9])*np.pi/180
                speed = float(row[10])
                timestamp = float(row[0])

                # shift timestamps relative to start time
                if time_offset is None:
                    time_offset = timestamp
                    timestamp = 0
                else:
                    timestamp -= time_offset

                x_data.append(x)
                y_data.append(y)
                yaw_data.append(yaw)
                speed_data.append(speed)
                timestamp_data.append(timestamp)
        
        return GPSData(
                x=np.array(x_data),
                y=np.array(y_data),
                yaw=np.array(yaw_data),
                speed=np.array(speed_data),
                timestamp=np.array(timestamp_data))
