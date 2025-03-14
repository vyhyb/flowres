import serial
import numpy as np
import time

class SensirionSDP610PressureSensor:
    def __init__(self, port="COM12", scale_factor=1200):
        self.port = port
        self.scale_factor = scale_factor
        self.sensor = None

    def open(self):
        self.sensor = serial.Serial(self.port)

    def close(self):
        if self.sensor and self.sensor.is_open:
            self.sensor.close()

    def __del__(self):
        self.close()

    def read_pressure(self, num_readings=1):
        #if not self.sensor:
        #    raise ValueError("Serial connection is not open.")

        readings = np.zeros(num_readings)
        for i in range(num_readings):
            self.sensor.reset_input_buffer()
            self.sensor.write(b"CD?\x00")
            time.sleep(0.005)  # Wait 5 ms for the sensor to process the reading
            waiting_bytes = self.sensor.in_waiting
            raw_data = self.sensor.read(waiting_bytes).decode().strip("\x00")

            try:
                pressure = int(raw_data)
                pressure /= self.scale_factor
                readings[i] = pressure
            except ValueError:
                # If the data cannot be cast to int, set the reading to NaN
                readings[i] = np.nan

        average_pressure = np.nanmean(readings)
        return average_pressure

# Usage example:
if __name__ == "__main__":
    pressure_sensor = SensirionSDP610PressureSensor()
    pressure_sensor.open_connection()
    try:
        average_pressure = pressure_sensor.read_pressure(num_readings=10)
        print(f"Average Pressure: {average_pressure:.2f}")
    finally:
        pressure_sensor.close_connection()