"""This module contains the functions for processing the data from
the ISO 9053-1 related measurements.
"""
import numpy as np 
from numpy import ndarray, savetxt
import csv
import os
from scipy.stats import linregress
from scipy.stats._stats_mstats_common import LinregressResult
from dataclasses import dataclass, asdict
from typing import Tuple
import logging
MULT_VOLUME = {
    'm3': 1,
    'l': 1e3,
    'cm3': 1e6,
    'mm3': 1e9
}

MULT_TIME = {
    'h': 60**2,
    'min': 60,
    's': 1
}

@dataclass
class Results:
    flow_rates: ndarray
    pressures: ndarray
    regression_slope: float
    regression_slope_err: float
    regression_intercept: float
    regression_intercept_err: float
    regression_rvalue: float
    flow_resistance: float
    flow_resistance_stderr: float
    specific_flow_resistance: float
    specific_flow_resistance_stderr: float
    flow_resistivity: float
    flow_resistivity_stderr: float
    velocity_min: float
    velocity_max: float
    surface_area: float
    surface_std: float
    thickness_mean: float
    thickness_std: float

    def export(self, filename='unknown', parent='.', delimiter=','):
        """Export the results to a CSV file with the given filename and delimiter.

        Parameters
        ----------
        filename : str, optional
            The name of the file to save the results to. Defaults to 'unknown'.
        parent : str, optional
            The parent directory to save the file to. Defaults to '.'.
        delimiter : str, optional
            The delimiter to use in the CSV file. Defaults to ','.
        """
        header='pressure'+delimiter+'flow_rate'
        data = np.stack((self.pressures, self.flow_rates), axis=1)
        savetxt(os.path.join(parent, filename+'_raw.csv'), data, delimiter=delimiter, header=header, comments='')
        
        results_dict = asdict(self)
        results_dict.pop('pressures')
        results_dict.pop('flow_rates')
        header_str = ''
        values_str = ''
        with open(os.path.join(parent, filename+'_results.csv'), 'w') as csvfile:
            for i, (k, v) in enumerate(results_dict.items()):
                header_str += k + delimiter
                values_str += f"{v:.5g}" + delimiter
            header_str = header_str.strip(delimiter)
            values_str = values_str.strip(delimiter)
            csvfile.write(header_str+"\n"+values_str)

def flow_rate_conversions(flow_rates, input_dim = 'l/min'):
    """Convert flow rates to m^3/s.

    Parameters
    ----------
    flow_rates : ndarray
        The array of flow rates to convert.
    input_dim : str, optional
        The input dimension of the flow rates. Defaults to 'l/min'.

    Returns
    -------
    rates : ndarray
        The array of flow rates converted to m^3/s.

    Raises
    ------
    ValueError
        If the input dimension is not implemented.
    """
    input_dim = input_dim.split('/')
    mult = 1

    if input_dim[0] in MULT_VOLUME.keys():
        mult /= MULT_VOLUME[input_dim[0]]
    else:
        raise ValueError(f"Unsupported volume unit: {input_dim[0]}.\nSupported units: {MULT_VOLUME.keys()}")

    if input_dim[1] in MULT_TIME.keys():
        mult /= MULT_TIME[input_dim[1]]
    else:
        raise ValueError(f"Unsupported time unit: {input_dim[1]}.\nSupported units: {MULT_TIME.keys()}")
    # print(flow_rates)
    flow_rates *= mult
    # print(mult)
    return flow_rates

def regression(flow_rates, pressures):
    """Perform a linear regression on the flow rates and pressures.

    Parameters
    ----------
    flow_rates : ndarray
        The array of flow rates.
    pressures : ndarray
        The array of pressures.

    Returns
    -------
    reg : LinregressResult
        The result of the linear regression.
    """
    reg = linregress(flow_rates, pressures)
    return reg

def airflow_resistance(reg):
    """Calculate the airflow resistance from the regression.

    Parameters
    ----------
    reg : LinregressResult
        The result of the linear regression.
    Returns
    -------
    flow_resistance : float
        The airflow resistance
    stderr : float
        The standard error of the regression.
    """
    flow_resistance = reg.slope
    stderr = reg.stderr
    return flow_resistance, stderr

def specific_airflow_resistance(flow_resistance, surface_area):
    """Calculate the specific airflow resistance.

    Parameters
    ----------
    flow_resistance : float
        The airflow resistance.
    surface_area : float
        The surface area of the specimen.

    Returns
    -------
    specific_flow_resistance : float
        The specific airflow resistance.
    """
    specific_flow_resistance = flow_resistance*surface_area
    return specific_flow_resistance

def airflow_resistivity(specific_flow_resistance, thickness):
    """Calculate the airflow resistivity.

    Parameters
    ----------
    specific_flow_resistance : float
        The specific airflow resistance.
    thickness : float
        The thickness of the specimen.

    Returns
    -------
    flow_resistivity : float
        The airflow resistivity
    """
    flow_resistivity = specific_flow_resistance / thickness
    return flow_resistivity

def velocity_boundaries(flow_rates, surface_area):
    """Calculate the minimum and maximum velocity boundaries.

    Parameters
    ----------
    flow_rates : ndarray
        The array of flow rates.
    surface_area : float
        The surface area of the specimen.

    Returns
    -------
    vel_bound : Tuple[float, float]
        The minimum and maximum velocity boundaries
    """
    minimum = flow_rates.min() / surface_area
    maximum = flow_rates.max() / surface_area
    vel_bound = (minimum, maximum)
    return vel_bound

def iso_calculation(flow_rates, pressures, surface_area, thickness, input_flow_dim = 'l/min'):
    """Perform the ISO 9053-1 calculation.

    Parameters
    ----------
    flow_rates : ndarray
        The array of flow rates.
    pressures : ndarray
        The array of pressures.
    surface_area : float or ndarray
        The surface area of the specimen.
    thickness : float or ndarray
        The thickness of the specimen.
    input_flow_dim : str, optional
        The input dimension of the flow rates. Defaults to '
        l/min'.

    Returns
    -------
    results : Results
        The results of the calculation.
    """
    flr = flow_rates.copy()
    flow_rates_m3 = flow_rate_conversions(flr, input_dim=input_flow_dim)
    reg = regression(flow_rates=flow_rates_m3, pressures=pressures)
    flow_resistance, stderr = airflow_resistance(reg)
    # old: resistance_std = np.sqrt(len(flow_rates)) * stderr
    resistance_std = stderr
    if not (
            isinstance(surface_area, (int, float)) 
            and isinstance(thickness, (int, float))
        ):
        surface_area = np.array(surface_area)
        thickness = np.array(thickness)
        surface_mean = np.mean(surface_area)
        surface_std = np.std(surface_area)
        thickness_mean = np.mean(thickness)
        thickness_std = np.std(thickness)
    elif not isinstance(surface_area, (int, float)):
        surface_area = np.array(surface_area)
        surface_mean = np.mean(surface_area)
        surface_std = np.std(surface_area)
        thickness_mean = thickness
        thickness_std = 0
    elif not isinstance(thickness, (int, float)):
        thickness = np.array(thickness)
        surface_mean = surface_area
        surface_std = 0
        thickness_mean = np.mean(thickness)
        thickness_std = np.std(thickness)
    else: 
        surface_mean = surface_area
        surface_std = 0
        thickness_mean = thickness
        thickness_std = 0
        
    specific_flow_resistance = specific_airflow_resistance(flow_resistance, surface_mean)
    flow_resistivity = airflow_resistivity(specific_flow_resistance, thickness_mean)

    # error propagation
    sfr_err1 = surface_mean * resistance_std
    sfr_err2 = flow_resistance * surface_std
    specific_flow_resistance_stderr = (
        (sfr_err1) ** 2
        + (sfr_err2) ** 2
    ) ** 0.5
    logging.info(f"Error propagation members for spec. airf. resistance: {sfr_err1:.2f}(slope);{sfr_err2:.2f}(surface)")
    fr_err1 = surface_mean / thickness_mean * resistance_std
    fr_err2 = flow_resistance / thickness_mean * surface_std
    fr_err3 = -1 * flow_resistance * surface_mean / thickness_mean**2 * thickness_std
    flow_resistivity_stderr = (
        (fr_err1) ** 2
        + (fr_err2) ** 2
        + (fr_err3) ** 2
    ) ** 0.5    
    logging.info(f"Error propagation members for flow resistivity: {fr_err1:.2f}(slope);{fr_err2:.2f}(surface);{fr_err3:.2f}(thickness)")
    vel_boundaries = velocity_boundaries(flow_rates_m3, surface_mean)

    results = Results(
        flow_rates=flow_rates_m3,
        pressures=pressures,
        regression_slope=reg.slope,
        regression_slope_err=reg.stderr,
        regression_intercept=reg.intercept,
        regression_intercept_err=reg.intercept_stderr,
        regression_rvalue=reg.rvalue,
        flow_resistance=flow_resistance,
        flow_resistance_stderr=stderr,
        specific_flow_resistance=specific_flow_resistance,
        specific_flow_resistance_stderr=specific_flow_resistance_stderr,
        flow_resistivity=flow_resistivity,
        flow_resistivity_stderr=flow_resistivity_stderr,
        velocity_min=vel_boundaries[0],
        velocity_max=vel_boundaries[1],
        surface_area = surface_mean,
        surface_std = surface_std,
        thickness_mean = thickness_mean,
        thickness_std = thickness_std,
    )
    return results
