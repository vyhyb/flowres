"""Flowres - A package for flow resistivity measurements according to ISO 9053-1

This package provides a set of tools for measuring flow resistivity of
porous materials according to ISO 9053-1. The package assumes three main
components, which are defined as protocols:

- PressureSensor: A protocol for pressure sensors
- FlowSensor: A protocol for flow sensors
- FlowActuator: A protocol for flow actuators

Having these protocols defined, the package provides a set of functions
for measuring flow resistivity, including error propagation, data
processing, and exporting data to CSV files.

The repository contains also example implementations of the protocols for
specific sensors and actuators, but mostly, that part is left for the user.

## Installation

It is currently not possible to install this library using `pip` or `conda`,
please use the latest [released package](https://github.com/vyhyb/flowres/releases)
instead and install using [`pip` locally](https://packaging.python.org/en/latest/tutorials/installing-packages/).

## Documentation

Documentation can be found [here](https://vyhyb.github.io/flowres/).

## Usage

An example script based on the provided example implementations of the protocols.

```python
from flowres import measurement_loop, export_csv, flow_rate_array, iso_calculation
from sensors import FestoFlowActuator, TSI4100FlowMeter, PressureSensor
import os
import numpy as np
from time import strftime

# initiation of the sensor
flow_sensor = TSI4100FlowMeter("COM1")
flow_actuator = FestoFlowActuator()
pressure_sensor = PressureSensor()

# specimen definition
name = "test"
timestamp = strftime("%y-%m-%d_%H-%M")
radius = 0.0225

start_flow = 0.5
end_flow = 1.5
steps = 100
two_way = True
target_flow_rates = flow_rate_array(start_flow, end_flow, steps, two_way)

# measurement
try:
    flow_sensor.open()
    pressure_sensor.open()
    
    pressure, flow_rate = measurement_loop(pressure_sensor, flow_sensor, flow_actuator, target_flow_rates)
    data = np.stack((pressure, flow_rate), axis=1)
    export_csv(data, f"{timestamp}_{name}.csv")
finally:
    flow_sensor.close()
    pressure_sensor.close()

# post processing
diameter = radius*2
surface_area = np.pi*radius**2
results = iso_calculation(flow_rates=flow_rate, pressures=pressure, surface_area=surface_area)
results.export(filename=f"{timestamp}_{name}")

# plot the results together with the regression
import matplotlib.pyplot as plt
from scipy.stats import linregress

flow_rate_m3s = flow_rate / 1000 / 60 # L/min to m^3/s
velocity_mms = flow_rate_m3s / (np.pi*radius**2) * 1000 # m^3/s to mm/s

# linear regression
reg = linregress(pressure, velocity_mms)
pressure_reg = np.array([0, pressure[-1]])
velocity_reg = pressure_reg*reg.slope
# std envelopes
velocitystd_reg_top = pressure_reg*(reg.slope+reg.stderr) + reg.intercept+reg.intercept_stderr
velocitystd_reg_bottom = pressure_reg*(reg.slope-reg.stderr) + reg.intercept-reg.intercept_stderr

# plots
plt.scatter(pressure,velocity_mms, alpha=0.5, label="measured points");
plt.fill_between(pressure_reg, velocitystd_reg_top, velocitystd_reg_bottom, alpha=0.7, label=f"regression +- stderr")
plt.plot(pressure_reg, velocity_reg, label=f"regression\n(offset {reg.intercept:.3f} +- {reg.intercept_stderr:.3f})")
plt.legend()
plt.xlabel("pressure[Pa]")
plt.ylabel("velocity[mm/s]")
plt.savefig(f"{timestamp}_{name}.png")
plt.show()
```

Refer to the `measurement.ipynb` notebook for a more detailed example.

## Author

- [David Jun](https://www.fce.vutbr.cz/o-fakulte/lide/david-jun-12801/)
  
  PhD student at [Brno University of Technology](https://www.vutbr.cz/en/).

## Contributing

Pull requests are welcome. For any changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)

Flowres - A package for flow resistivity measurements according to ISO 9053-1
Copyright (C) 2025 David Jun

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

## References

[1] ČSN EN ISO 9053-1 - Akustika - Stanovení odporu proti proudění vzduchu - část 1: Metoda statického proudění vzduchu, manual, Praha., 2019.
"""
from .flow_pressure_measurement import measurement_loop, measurement, voltage_guess
from .misc import export_csv, flow_rate_array, final_beep
from .processing import iso_calculation
