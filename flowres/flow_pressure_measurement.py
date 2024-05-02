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

class PressureSensor(Protocol):
    ...
    def read_pressure(self, num_readings=1) -> float:
        ...

class FlowSensor(Protocol):
    ...
    def read_flow(self, num_readings=1) -> float:
        ...

class FlowActuator(Protocol):
    ...
    def set_voltage(self, voltage):
        ...
    def change_voltage(self, voltage_change):
        ...

class PIDController(Protocol):
    ...
    def compute(self, current_value):
        ...

def measurement_loop(
        pressure_sensor : PressureSensor,
        flow_sensor : FlowSensor,
        flow_actuator : FlowActuator,
        flow_rates : List[float]
        ) -> Tuple[np.ndarray]:
    flow_rates = np.array(flow_rates)
    pressure_arr = np.empty(len(flow_rates))
    flow_rate_arr = np.empty(len(flow_rates))
    for idx, f in np.ndenumerate(flow_rates):
        logging.debug(f"Measurement for the target flow of {f:.3f} l/min started. {(idx[0]+1)}/{len(flow_rates)}")
        pressure, flow_rate = measurement(pressure_sensor, flow_sensor, flow_actuator, f)
        pressure_arr[idx] = pressure
        flow_rate_arr[idx] = flow_rate
        progressbar(idx[0], len(flow_rates), 50, '■')
    return pressure_arr, flow_rate_arr
    
def measurement_pid( #just idea, not tested
        pressure_sensor : PressureSensor,
        flow_sensor : FlowSensor,
        flow_actuator : FlowActuator,
        pid_controller : PIDController,
        f : float,
        max_iterations=20,
        tolerance_percent=5
        ) -> Tuple[float]:
    setpoint_flow = f
    tolerance = tolerance_percent * f / 100
    init_voltage = initial_voltage_guess(setpoint_flow)
    sensor.set_voltage(init_voltage)

    iterations = 0
    while iterations < max_iterations:
        current_flow = flow_meter.read_flow()
        if current_flow is not None:
            error = setpoint_flow - current_flow
            pid_output = pid_controller.compute(error)
            too_high_flow = (flow_actuator.current_voltage + pid_output) < flow_actuator.min_voltage
            too_low_flow = (flow_actuator.current_voltage + pid_output) > flow_actuator.max_voltage
            if (abs(pid_output) > max_output) or too_high_flow or too_low_flow:
                iteration = max_iterations
                print(".", end="")
            flow_actuator.change_voltage(pid_output)
            
                
            if abs(current_flow - setpoint_flow) < tolerance:
                break

        time.sleep(pid.sample_time)
        iteration += 1
    sleep(4)
    pressure = pressure_sensor.read_pressure(10)
    flow_rate = flow_sensor.read_flow(100)
    return pressure, flow_rate

def measurement(
        pressure_sensor : PressureSensor,
        flow_sensor : FlowSensor,
        flow_actuator : FlowActuator,
        target_flow_rate : float,
        max_iterations=100,
        tolerance_stability=0.001
        ) -> Tuple[float]:
    voltage = voltage_guess(target_flow_rate)
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

def voltage_guess(target_flow, calibration_array=CALIBRATION_ARRAY):
    calibrated_flow = calibration_array[1]
    calibrated_voltage = calibration_array[0]
    difference_array = np.absolute(calibrated_flow-target_flow)
    index = difference_array.argmin()
    logging.debug(f"Voltage guess read from the calibration file (idx:{index}, voltage:{calibrated_voltage[index]:.3f})")

    voltage = calibrated_voltage[index]
    return voltage