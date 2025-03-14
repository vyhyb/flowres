import hid


class FestoFlowActuator:
    """A class representing a Festo flow actuator.

    This class provides methods to control the voltage of the actuator.

    Parameters
    ----------
    vendor_id : int, optional
        The vendor ID of the actuator. Defaults to 0x04d8.
    product_id : int, optional
        The product ID of the actuator. Defaults to 0x0056.
    max_voltage : float, optional
        The maximum voltage supported by the actuator. Defaults to 5.0.
    min_voltage : float, optional
        The minimum voltage supported by the actuator. Defaults to 3.5.
    initial_voltage : float, optional
        The initial voltage of the actuator. Defaults to 3.5.

    Raises
    ------
    ValueError
        If the initial voltage is outside the valid range.

    Attributes
    ----------
    vendor_id : int
        The vendor ID of the actuator.
    product_id : int
        The product ID of the actuator.
    max_voltage : float
        The maximum voltage supported by the actuator.
    min_voltage : float
        The minimum voltage supported by the actuator.
    actuator : hid.device
        The HID device object representing the actuator.
    current_voltage : float
        The current voltage of the actuator.
    """

    def __init__(
        self, vendor_id=0x04d8, product_id=0x0056, max_voltage=5.0, min_voltage=3.5, initial_voltage=3.5
    ):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.max_voltage = max_voltage
        self.min_voltage = min_voltage
        self.actuator = hid.device()
        self.actuator.open(vendor_id, product_id)
        self.actuator.set_nonblocking(1)

        if initial_voltage < 0 or initial_voltage > self.max_voltage:
            raise ValueError(
                f"Initial voltage must be between 0 and {self.max_voltage} volts."
            )
        self.current_voltage = initial_voltage

        self.set_voltage(initial_voltage)
    def __del__(self):
        self.actuator.close()

    def set_voltage(self, voltage):
        """Set the voltage of the actuator.

        Args:
            voltage (float): The voltage to set.

        Raises:
            ValueError: If the voltage is outside the valid range.
        """
        if voltage < self.min_voltage:
            voltage = self.min_voltage
        elif voltage > self.max_voltage:
            voltage = self.max_voltage

        value = int(voltage * 4096 / self.max_voltage)
        string = (
            b"\x00\x811"
            + bytes(("0" + hex(value)[2:])[-3:], encoding="ascii")
            + b"\x00" * 59
        )
        self.actuator.write(string)
        self.current_voltage = voltage

    def change_voltage(self, voltage_change):
        """Change the voltage of the actuator by a specified amount.

        Args:
            voltage_change (float): The amount by which to change the voltage.
        """
        new_voltage = self.current_voltage + voltage_change
        self.set_voltage(new_voltage)


# Usage example:
if __name__ == "__main__":
    flow_actuator = FestoFlowActuator(initial_voltage=3.0)

    try:
        print(f"Current Voltage: {flow_actuator.current_voltage:.2f} volts")

        flow_actuator.change_voltage(1.0)  # Increase voltage by 1.0 volt
        print(f"New Voltage: {flow_actuator.current_voltage:.2f} volts")

    finally:
        flow_actuator.set_voltage(0.0)  # Set voltage back to 0 before closing