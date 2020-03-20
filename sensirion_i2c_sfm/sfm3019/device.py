# -*- coding: utf-8 -*-
# (c) Copyright 2020 Sensirion AG, Switzerland

from __future__ import absolute_import, division, print_function
from .commands import Sfm3019I2cCmdReadMeas, \
        Sfm3019I2cCmdReadProductIdentifierAndSerialNumber, \
        Sfm3019I2cCmdStartMeasO2, Sfm3019I2cCmdStartMeasAir, \
        Sfm3019I2cCmdStartMeasAirO2Mix


class Sfm3019I2cSensorBridgeDevice:
    """
    SFM3019 I²C device class to allow executing I²C commands.
    """

    MeasurementModes = {
        'O2': Sfm3019I2cCmdStartMeasO2,
        'Air': Sfm3019I2cCmdStartMeasAir,
        'AirO2Mix': Sfm3019I2cCmdStartMeasAirO2Mix,
    }

    def __init__(self, sensor_bridge, sensor_bridge_port, slave_address=0x2E):
        """
        Constructs a new SFM3019 I²C device.

        :param ~sensirion_i2c_driver.connection.I2cConnection connection:
            The I²C connection to use for communication.
        :param byte slave_address:
            The I²C slave address, defaults to 0x2E.
        """
        self._sensor_bridge = sensor_bridge
        self._sensor_bridge_port = sensor_bridge_port
        self._slave_address = slave_address

    def _execute(self, command):
        """
        Perform read and write operations of an I²C command.

        :param byte slave_address:
            The slave address of the device to communicate with.
        :param ~sensirion_i2c_driver.command.I2cCommand command:
            The command to execute.
        :return:
            - In single channel mode: The interpreted data of the command.
            - In multi-channel mode: A list containing either interpreted data
              of the command (on success) or an Exception object (on error)
              for every channel.
        :raise:
            In single-channel mode, an exception is raised in case of
            communication errors.
        """
        response = self._sensor_bridge.transceive_i2c(self._sensor_bridge_port,
                address=self._slave_address,
                tx_data=command.tx_data,
                rx_length=command.rx_length,
                timeout_us=command.timeout * 1e6,
        )
        return command.interpret_response(response)

    def read_product_identifier_and_serial_number(self):
        """
        Read the product identifier and serial number.

        :return:
            The product identifier (4b) and serial number (12b), both formatted
            as hex string.
        :rtype:
            tuple
        """
        hex_res = self._execute(Sfm3019I2cCmdReadProductIdentifierAndSerialNumber())
        return hex_res[0:4], hex_res[4:]

    def start_continuous_measurement(self, measure_mode='Air',
                                     air_o2_mix_fraction_permille=None):
        """
        Start a continuous measurement with the specified gas

        :param string measure_mode:
            Configure the type of measurement to perform. Check the datasheet
            for more information. Available are 'Air', 'O2' and 'AirO2Mix'
        :param int air_o2_mix_fraction_permille:
            Fraction (in permille 0-1000) of O2 contained in the Air/O2 Mix.
            This parameter is only used when measure_mode is 'AirO2Mix'.
        """
        if measure_mode == 'AirO2Mix':
            mm = Sfm3019I2cSensorBridgeDevice.MeasurementModes[measure_mode]
            return self._execute(mm(air_o2_mix_fraction_permille))

        return self._execute(Sfm3019I2cSensorBridgeDevice.MeasurementModes[measure_mode]())

    def read_continuous_measurement(self):
        """
        Read a single measurement from the running measurement

        :return:
            The measured flow and temperature

            - flow (int) -
              flow adc ticks.
            - temperature (int) -
              Temperature adc ticks.
        :rtype:
            tuple
        """
        return self._execute(Sfm3019I2cCmdReadMeas)
