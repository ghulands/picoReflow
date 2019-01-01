#!/usr/bin/python
import logging

from Adafruit_MAX31855 import MAX31855
import driver


class MAX31855SPI(driver.TemperatureDriver):
    """Python driver for [MAX38155 Cold-Junction Compensated Thermocouple-to-Digital Converter](http://www.maximintegrated.com/datasheet/index.mvp/id/7273)
     Requires:
     - adafruit's MAX31855 SPI-only device library
    """
    def __init__(self, options):
        super().__init__(options)
        self.max31855 = MAX31855.MAX31855(spi=options['spi_dev'])
        self.log = logging.getLogger(__name__)

    def get_temperature(self, channel: int = 0) -> float:
        """Reads SPI bus and returns current value of thermocouple."""
        state = self.max31855.readState()
        self.log.debug("status %s" % state)
        if state['openCircuit']:
            raise driver.DriverError(driver.ERROR_NO_CONNECTION, 'Thermocouple not connected.')
        elif state['shortGND']:
            raise driver.DriverError(driver.ERROR_SHORT_TO_GROUND, 'Thermocouple shorted to ground.')
        elif state['shortVCC']:
            raise driver.DriverError(driver.ERROR_SHORT_TO_VCC, 'Thermocouple shorted to VCC.')
        elif state['fault']:
            raise driver.DriverError(driver.ERROR_UNKNOWN, 'Unknown Error.')
        return self.max31855.readLinearizedTempC()
