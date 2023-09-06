import logging
import time

from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_sensorbridge import SensorBridgePort, SensorBridgeShdlcDevice

from sensirion_sensorbridge_i2c_sfm.sfm3000 import Sfm3000I2cSensorBridgeDevice, MeasurementMode
from sensirion_sensorbridge_i2c_sfm.sfm3000.sfm3000_constants import SFM3000_DEFAULT_I2C_FREQUENCY, \
    SFM3000_DEFAULT_VOLTAGE

logging.basicConfig(format='%(asctime)s [%(levelname)s] %(message)s', level=logging.ERROR)

# Connect to the SensorBridge with default settings:
#  - baudrate:      460800
#  - slave address: 0
with ShdlcSerialPort(port='/dev/ttyUSB0', baudrate=460800) as port: #baudrate=460800
    # Initialize Sensorbridge
    bridge = SensorBridgeShdlcDevice(ShdlcConnection(port), slave_address=0)
    print("SensorBridge SN: {}".format(bridge.get_serial_number()))

    # Configure SensorBridge port 1 for SFM
    bridge.set_i2c_frequency(SensorBridgePort.ONE, frequency=SFM3000_DEFAULT_I2C_FREQUENCY)
    bridge.set_supply_voltage(SensorBridgePort.ONE, voltage=SFM3000_DEFAULT_VOLTAGE)
    bridge.switch_supply_on(SensorBridgePort.ONE)

    # Create SFM device
    sfm3000 = Sfm3000I2cSensorBridgeDevice(bridge, SensorBridgePort.ONE, slave_address=0x40) # 02E
    
    # Define gas (or gas mixes)
    measure_mode = MeasurementMode.Air
    permille = 200  # only applies for gas mixes

    # Initialize sensor:
    # 1.) Stop any running measurement
    # 2.) Request scale factors and unit set on sensor
    
    
    sfm3000.initialize_sensor(measure_mode)
    
    
    # Read out product information
    pid, sn = sfm3000.read_product_identifier_and_serial_number()
    print("Sfm3000 SN: {}".format(sn))
    #print("Flow unit of sensor: {} (Volume at temperature in degree Centigrade)".format(sfm3000.flow_unit))
    
    # Start measurements
    sfm3000.start_continuous_measurement(measure_mode, air_o2_mix_fraction_permille=permille)
    print('yay')
    # Read them out continuously
    while True:
        time.sleep(0.1)
        print("Flow: {}, Temperature: {}".format(*sfm3000.read_continuous_measurement()))
    