import serial
import numpy as np
import time
import logging

class TSI4100FlowMeter:
    def __init__(self, port, baudrate=38400, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def open(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            # Wait for the meter to initialize after connection
            time.sleep(1)
            logging.debug("Connected to the TSI 4100 Series flow rate meter.")    
        except serial.SerialException as e:
            print("Failed to connect to the TSI 4100 Series flow rate meter: {}".format(e))
            self.disconnect()

    def close(self):
        if self.serial:
            self.serial.close()
            logging.debug("Disconnected from the TSI 4100 Series flow rate meter.")

    def __del__(self):
        self.close()

    def read_flow(self, num_readings=100):
        str_readings = ("000"+str(num_readings))[-4:].encode()
        try:
            self.serial.write(b"DAFxx"+str_readings+b"\r")  # Send the 'C' command to request flow value
            response = self.serial.readline().decode().strip("\r\n").strip()
            if response == "OK":
                response = self.serial.readline().decode().strip("\r\n").strip().split(",")
                response = np.array(response)
                flow_value = response.astype(np.float32).mean()
                return flow_value
            else:
                print("Error: Unexpected response from the meter.")
                return None
        except (ValueError, serial.SerialException) as e:
            print("Error reading flow value: {}".format(e))
            return None

# Usage example:
if __name__ == "__main__":
    port_name = "COM1"
    flow_meter = TSI4100FlowMeter(port_name)
    flow_meter.connect()

    try:
        while flow_meter.serial:
            flow_value = flow_meter.read_flow()
            if flow_value is not None:
                print("Current flow value: {} L/min".format(flow_value))
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    flow_meter.disconnect()