"""Module for the flow and pressure measurement using the flow actuator, 
pressure sensor and flow sensor. The module provides functions to perform
the measurement and calibration of the flow actuator for reaching a target flow.
"""
import numpy as np
from typing import Protocol, List, Tuple
import os
from time import sleep
import logging
from flowres.misc import progressbar

# logging.basicConfig(level=logging.DEBUG)

try:
    CALIBRATION_ARRAY = np.load(os.path.join(os.path.dirname(__file__), "dac_calibration.npy")).T
except Exception:
    print("Unable to load calibration data for the flow actuator.")

#### PROTOCOLS TO BE IMPLEMENTED FOR EACH SETUP ####
class PressureSensor(Protocol):
    """Protocol for pressure sensors."""
    ...
    def read_pressure(self, num_readings=1) -> float:
        """Read the pressure from the sensor with the given number 
        of averaged readings.
        """
        ...

class FlowSensor(Protocol):
    """Protocol for flow sensors."""
    ...
    def read_flow(self, num_readings=1) -> float:
        """Read the flow from the sensor with the given number
        of averaged readings.
        """
        ...

class FlowActuator(Protocol):
    """Protocol for flow actuators."""
    ...
    def set_voltage(self, voltage):
        """Set the voltage of the actuator to the given value."""
        ...
    def change_voltage(self, voltage_change):
        """Change the voltage of the actuator by the given amount."""
        ...

#### FUNCTIONS ####
def measurement_loop(
        pressure_sensor : PressureSensor,
        flow_sensor : FlowSensor,
        flow_actuator : FlowActuator,
        flow_rates : List[float],
        calibration_array=CALIBRATION_ARRAY
        ) -> Tuple[np.ndarray]:
    """Perform the measurement loop for the given flow rates using the
    pressure sensor, flow sensor and flow actuator. The function returns
    the arrays of pressure and flow rates.
    """    
    flow_rates = np.array(flow_rates)
    pressure_arr = np.empty(len(flow_rates))
    flow_rate_arr = np.empty(len(flow_rates))
    for idx, f in np.ndenumerate(flow_rates):
        logging.debug(f"Measurement for the target flow of {f:.3f} l/min started. {(idx[0]+1)}/{len(flow_rates)}")
        pressure, flow_rate = measurement(
            pressure_sensor,
            flow_sensor,
            flow_actuator,
            f,
            calibration_array=calibration_array
            )
        pressure_arr[idx] = pressure
        flow_rate_arr[idx] = flow_rate
        progressbar(idx[0], len(flow_rates), 50, '■')
    return pressure_arr, flow_rate_arr

def measurement(
        pressure_sensor : PressureSensor,
        flow_sensor : FlowSensor,
        flow_actuator : FlowActuator,
        target_flow_rate : float,
        max_iterations=100,
        tolerance_stability=0.001,
        calibration_array=CALIBRATION_ARRAY
        ) -> Tuple[float]:
    """Perform the measurement for the given target flow rate using the
    pressure sensor, flow sensor and flow actuator. The function returns
    the pressure and flow rate.
    """
    voltage = voltage_guess(target_flow_rate, calibration_array)
    flow_actuator.set_voltage(voltage)
    sleep(0.05)
    last_flow_rate = flow_sensor.read_flow(100)
    tolerance = tolerance_stability * last_flow_rate
    sleep(0.05)
    iterations = 0
    while iterations < max_iterations:
        current_flow_rate = flow_sensor.read_flow(100)
        error = abs(last_flow_rate - current_flow_rate)
        if error < tolerance:
            logging.debug(f"Flow stabilized after {iterations+1} iterations "+
                          f"with the difference in successive readings {error/last_flow_rate*100:.3f} % of the last one.")
            break
        iterations += 1
        last_flow_rate = current_flow_rate
        sleep(0.05)
    if iterations == max_iterations:
        logging.debug(f"Flow never stabilized (after {iterations} iterations).")
    pressure = pressure_sensor.read_pressure(10)
    flow_rate = flow_sensor.read_flow(100)
    logging.debug(f"Everything read.")
    return pressure, flow_rate

def voltage_guess(
        target_flow: float,
        calibration_array: np.ndarray = CALIBRATION_ARRAY
        ) -> float:
    """Guess the voltage for the flow actuator to reach the target flow
    based on the calibration data.
    """
    calibrated_flow = calibration_array[1]
    calibrated_voltage = calibration_array[0]
    difference_array = np.absolute(calibrated_flow-target_flow)
    index = difference_array.argmin()
    logging.debug(f"Voltage guess read from the calibration file (idx:{index}, voltage:{calibrated_voltage[index]:.3f})")

    voltage = calibrated_voltage[index]
    return voltage

def calibrate_flow_actuator(
        flow_actuator : FlowActuator,
        flow_sensor : FlowSensor,
        voltage_range : Tuple[float, float],
        steps = 100,
        filename="dac_calibration.npy"
        ) -> np.ndarray:
    """Calibrate the flow actuator using the flow sensor for the given
    voltage range and number of steps. The function saves the calibration
    data to the file and returns the calibration data.
    """
    start_voltage, end_voltage = voltage_range
    voltages = np.linspace(start_voltage, end_voltage, steps)
    flow_rates = np.empty(steps)
    for idx, v in np.ndenumerate(voltages):
        flow_actuator.set_voltage(v)
        sleep(0.1)
        flow_rates[idx] = flow_sensor.read_flow(100)
        progressbar(idx[0], steps, 50, '■')
    np.save(filename, np.stack((voltages, flow_rates)))
    return np.stack((voltages, flow_rates))