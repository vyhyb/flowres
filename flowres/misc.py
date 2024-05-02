from numpy import savetxt, linspace, concatenate
import numpy as np
from flowres.processing import Results
from dataclasses import asdict
import os
import csv
from time import sleep
from typing import Protocol

class FlowActuator(Protocol):
    def set_voltage(self, voltage):
        ...
    
def progressbar(current_value,total_value,bar_lengh,progress_char): 
    current_value += 1
    percentage = int((current_value/total_value)*100)                                                # Percent Completed Calculation 
    progress = int((bar_lengh * current_value ) / total_value)                                       # Progress Done Calculation 
    loadbar = "Progress: [{:{len}}]{}%".format(progress*progress_char,percentage,len = bar_lengh)    # Progress Bar String
    if current_value == total_value:
        loadbar = "Progress: [{:{len}}]{}".format(bar_lengh*progress_char,'Done',len = bar_lengh) 
    print(loadbar, end='\r')

def export_csv(data, filename, parent=".", delimiter=","):
    header='pressure'+delimiter+'flow_rate'
    savetxt(os.path.join(parent, filename), data, delimiter=delimiter, header=header, comments='')

def flow_rate_array(start, stop, steps, two_way=False):
    flow_rates = linspace(start, stop, steps)
    if two_way:
        flow_rates = concatenate((flow_rates[::2], flow_rates[-2::-2]))
    return flow_rates

def export_results(results: Results, filename, parent=".", delimiter=","):
    results.export(filename, parent, delimiter)

def final_beep(flow_actuator: FlowActuator):
    flow_actuator.set_voltage(0)
    sleep(0.3)
    flow_actuator.set_voltage(4.5)
    sleep(0.3)
    flow_actuator.set_voltage(0)
    sleep(0.6)
    flow_actuator.set_voltage(4.5)