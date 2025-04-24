import os
import time
import threading
import socket
import random
import numpy as np
import csv
from datetime import datetime

ARDUINO_IP = "192.168.2.108"
ARDUINO_PORT = 5000
ANT_IP = "127.0.0.1"
ANT_PORT = 8080
POLLING_INTERVAL = 0.01
SOCKET_TIMEOUT = 1

class BikeSensor:
    def __init__(self, arduino_ip=ARDUINO_IP, arduino_port=ARDUINO_PORT, 
                 ant_ip=ANT_IP, ant_port=ANT_PORT, 
                 polling_interval=POLLING_INTERVAL, timeout=SOCKET_TIMEOUT,
                 cutoff_frequency=5):
        # Arduino connection
        self.arduino_address = (arduino_ip, arduino_port)
        self.arduino_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.arduino_socket.settimeout(timeout)

        # ANT+ connection
        self.ant_address = (ant_ip, ant_port)
        self.ant_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ant_socket.bind(self.ant_address)
        self.ant_socket.settimeout(timeout)
        self.interval = polling_interval
        self._alive = True

        # Initialize values
        self._speed_original = 0            # Original speed from Arduino
        self._speed_original_previous = 0   # Previous speed from Arduino
        self._speed_calibrated = 0          # Calibrated speed
        self._speed = 0                     # Filtered speed
        self._steering = 0                  # Steering angle
        self.speed2 = 0                     # ANT+ speed
        self.cadence2 = 0                   # ANT+ cadence
        self.power2 = 0                     # ANT+ power 
        self.elapsedTime = 0                # ANT+ elapsed time
        self.distanceTraveled = 0           # ANT+ distance
        self.heartrate = 0                  # ANT+ heart rate
        self.lastPage10TS = 0               # ANT+ timestamp
        self.lastPage19TS = 0               # ANT+ timestamp
        self.grade = 5                      # Dummy Default grade
        self.baseSteering = 144             # Base steering value for calibration
        self.grade_data = []                # List to store grade data for averaging

        # Calibration
        self.speed_calibration_coeff = 1.0  # Default calibration coefficient
        self.last_ant_speed = 0             # Last ANT+ speed for calibration

        # Low-pass filter initialization (Experimental)
        self.cutoff_frequency = cutoff_frequency
        self.alpha = self.calculate_alpha(cutoff_frequency, polling_interval)
        self.prev_filtered_speed = 0

        # Data logging
        self.log_folder = "experiment_log"
        os.makedirs(self.log_folder, exist_ok=True)
        self.log_file = os.path.join(self.log_folder, f"bike_sensor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.write_log_header()

        # Start the communication threads
        self._arduino_thread = threading.Thread(target=self._arduino_loop)
        self._ant_thread = threading.Thread(target=self._ant_loop)
        self._arduino_thread.start()
        self._ant_thread.start()

    def calculate_alpha(self, cutoff_freq, dt):
        tau = 1 / (2 * np.pi * cutoff_freq)
        return dt / (tau + dt)

    def apply_low_pass_filter(self, new_value):
        filtered_value = self.alpha * new_value + (1 - self.alpha) * self.prev_filtered_speed
        self.prev_filtered_speed = filtered_value
        return filtered_value

    # Writing Header to CSV log file
    # This function is called only once when the log file is created
    def write_log_header(self):
        with open(self.log_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'Timestamp', 'Source', 'Speed_Arduino_Original', 'Speed_Arduino_Calibrated', 
                'Speed_Arduino_Filtered', 'Speed_ANT', 'Steering', 'Cadence', 'Power', 
                'ElapsedTime', 'DistanceTraveled', 'HeartRate', 'Grade', 'Calibration_Coefficient'
            ])

    # Logging data to CSV file
    # This function is called every time new data is received from Arduino or ANT+
    def log_data(self, timestamp, source, speed_arduino_original=None, speed_arduino_calibrated=None, 
                 speed_arduino_filtered=None, speed_ant=None, steering=None, cadence=None, 
                 power=None, elapsed_time=None, distance_traveled=None, heart_rate=None):
        with open(self.log_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp, source, speed_arduino_original, speed_arduino_calibrated, 
                speed_arduino_filtered, speed_ant, steering, cadence, power, elapsed_time, 
                distance_traveled, heart_rate, self.grade, self.speed_calibration_coeff
            ])

    def _arduino_loop(self):
        while self._alive:
            self.arduino_socket.sendto("foo".encode("utf-8"), self.arduino_address)
            try:
                rec_data, addr = self.arduino_socket.recvfrom(64)
                arduino_timestamp = time.time()  # Capture timestamp immediately after receiving Arduino data
                
                parsed_rec_data = rec_data.split(b', ')
                self._speed_original_previous = self._speed_original
                self._speed_original = float(parsed_rec_data[0]) * 30  # Original conversion
                self._steering = float(parsed_rec_data[1])
                if self._steering < 0:
                    self._steering = self._steering + 360
                self._steering = self._steering - self.baseSteering    # Calibration

                # Check for invalid speed values
                if self._speed_original > 2 and self._speed_original < 0.7 * self._speed:
                    self._speed_original = self._speed_original_previous
                
                # Apply speed calibration (Calibrate Arduino speed reading with ANT+ speed reading)
                self._speed_calibrated = self._speed_original * self.speed_calibration_coeff
                
                # Apply low-pass filter
                self._speed = self.apply_low_pass_filter(self._speed_calibrated)
                
                self.log_data(arduino_timestamp, 'Arduino', speed_arduino_original=self._speed_original, 
                             speed_arduino_calibrated=self._speed_calibrated, 
                             speed_arduino_filtered=self._speed, steering=self._steering)
            except socket.timeout:
                pass

    def _ant_loop(self):
        while self._alive:
            try:
                data, client_address = self.ant_socket.recvfrom(1024)
                data_str = data.decode()
                parsed_data = self.parse_data(data_str)
                
                self.speed2 = parsed_data.get('Speed', 0) * 3.6  # Convert to km/h
                self.cadence2 = parsed_data.get('Cadence', 0)
                self.power2 = parsed_data.get('Power', 0)
                self.elapsedTime = parsed_data.get('ElapsedTime', 0)
                self.distanceTraveled = parsed_data.get('DistanceTraveled', 0)
                # print(self.distanceTraveled) There is no data transmission for distanceTraveled in the ANT+ data
                self.heartrate = parsed_data.get('HeartRate', 0)
                self.lastPage10TS = parsed_data.get('LastPage10TS', 0)
                self.lastPage19TS = parsed_data.get('LastPage19TS', 0)
                
                # Update speed calibration coefficient
                if self.speed2 != self.last_ant_speed and self._speed_original != 0:
                    self.speed_calibration_coeff = self.speed2 / self._speed_original
                    self.last_ant_speed = self.speed2
                
                self.log_data(self.lastPage10TS, 'ANT', speed_ant=self.speed2, cadence=self.cadence2, 
                              power=self.power2, elapsed_time=self.elapsedTime, 
                              distance_traveled=self.distanceTraveled, 
                              heart_rate=self.heartrate)
                
                grade = self.grade 
                response = f"Grade:{grade}"
                self.ant_socket.sendto(response.encode(), client_address)
            except socket.timeout:
                pass

    @staticmethod
    def parse_data(data_str):
        data_dict = {}
        for item in data_str.split(','):
            key, value = item.split(':')
            data_dict[key] = float(value)
        return data_dict

    # Getter for Arduino speed and steering
    def get_speed_and_steering(self):
        return self._speed, self._steering
    
    # Getter for ANT+ speed, cadence, and power
    def get_speed2_cadence2_power2(self):
        return self.speed2, self.cadence2, self.power2
    
    # Logic for setting the grade
    # Values are averaged over 2 samples to reduce sudden jumps
    def set_grade(self, new_data):
        if -25 <= new_data <= 25:
            self.grade_data.append(new_data)
        if len(self.grade_data) == 2:
            average_grade = sum(self.grade_data) / len(self.grade_data)
            self.grade = average_grade  
            self.grade_data = []

    def get_grade(self):
        return self.grade

    def set_cutoff_frequency(self, new_cutoff_frequency):
        self.cutoff_frequency = new_cutoff_frequency
        self.alpha = self.calculate_alpha(new_cutoff_frequency, self.interval)
    
    # Stop the threads and close sockets
    def stop(self):
        self._alive = False
        self.arduino_socket.close()
        self.ant_socket.close()

    def __del__(self):
        self.stop()

if __name__ == "__main__":
    sensor = BikeSensor(cutoff_frequency=2)  # Adjust cutoff frequency as needed
    try:
        while True:
            speed, steering = sensor.get_speed_and_steering()
            speed2, cadence2, power2 = sensor.get_speed2_cadence2_power2()
            # Uncomment the following lines for debugging
            #print(f"Arduino - Speed (Filtered): {speed:.2f}, Steering: {steering:.2f}")
            #print(f"ANT+ - Speed: {speed2:.2f} km/h, Cadence: {cadence2:.2f} RPM, Power: {power2:.2f} W")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping sensor...")
        sensor.stop()