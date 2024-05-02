import hid
import numpy as np
import time
from typing import Protocol

class FlowActuator(Protocol):
    ...

    def read_voltage(self):
        ...
        
# PID Controller Class
class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0, sample_time=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error * self.sample_time
        derivative = (error - self.last_error) / self.sample_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

def evaluate_best_pid_parameters(flow_meter, flow_actuator, flow_rates, initial_voltage=4.5, max_output=3, sample_time=1):
    best_params = {'kp': 0.0, 'ki': 0.0, 'kd': 0.0}
    min_avg_error = float("inf")

    kp_range = np.linspace(0.5, 1, 5)
    ki_range = np.linspace(0.00, 0.05, 5)
    #kd_range = np.linspace(-0.0005, 0.0005, 3)
    kd_range = [0]
    for flow_rate in flow_rates:
        for kd in kd_range:
            for ki in ki_range:
                for kp in kp_range:                
                    flow_actuator.set_voltage(initial_voltage)  # Start with zero voltage
                    time.sleep(2)  # Allow time for the system to stabilize
                    pid = PIDController(kp, ki, kd, sample_time=sample_time)  # Create a new PID controller for each flow rate
                    setpoint_flow = flow_rate
            
                    max_iterations = 10
                    tolerance = 0.05*flow_rate  # Tolerance for reaching the setpoint flow
                    iteration = 0
                    total_error = 0.0
            
                    try:
                        while iteration < max_iterations:
                            current_flow = flow_meter.read_flow()
                            if current_flow is not None:
                                error = setpoint_flow - current_flow
                                pid_output = pid.compute(error)
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
            
                            # Accumulate the error for each iteration
                            total_error += abs(error)
            
                        # Calculate the average error for the current flow rate
                        avg_error = total_error / max_iterations
            
                        # Update the best parameters if the current flow rate has the smallest average error
                        if avg_error < min_avg_error:
                            min_avg_error = avg_error
                            best_params['kp'] = pid.kp
                            best_params['ki'] = pid.ki
                            best_params['kd'] = pid.kd
            
                    except KeyboardInterrupt:
                        pass

    flow_actuator.set_voltage(initial_voltage)  # Set voltage back to 0 before closing

    return best_params

# Flow Measurement Loop with PID Control
def flow_measurement_loop(flow_meter, flow_actuator, setpoint_flow, kp, ki, kd):
    pid = PIDController(kp, ki, kd, setpoint_flow)
    max_iterations = 100
    tolerance = 0.1  # Tolerance for reaching the setpoint flow
    iteration = 0

    try:
        while iteration < max_iterations:
            current_flow = flow_meter.read_flow()
            if current_flow is not None:
                print(f"Current flow value: {current_flow:.2f} L/min")

                pid_output = pid.compute(current_flow)
                print(f"PID Output: {pid_output:.2f} V")

                flow_actuator.change_voltage(pid_output)

                if abs(current_flow - setpoint_flow) < tolerance:
                    print(f"Setpoint flow ({setpoint_flow} L/min) reached.")
                    break

            time.sleep(pid.sample_time)
            iteration += 1

    except KeyboardInterrupt:
        pass

    flow_actuator.set_voltage(0.0)  # Set voltage back to 0 before closing

# Usage example:
if __name__ == "__main__":
    flow_meter = TSI4100FlowMeter(port_name)
    flow_meter.connect()

    flow_actuator = FlowActuator(initial_voltage=3.0)

    try:
        if flow_meter.serial:
            # Define the range of flow rates for auto-tuning
            flow_rates = [5.0, 10.0, 15.0]

            kp, ki, kd = ziegler_nichols_ultimate_gain_tuning(flow_meter, flow_actuator, flow_rates)
            print(f"Ideal PID Parameters: kp={kp:.2f}, ki={ki:.2f}, kd={kd:.2f}")

    finally:
        flow_meter.disconnect()

