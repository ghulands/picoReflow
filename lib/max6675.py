#!/usr/bin/python
import time

import RPi.GPIO as GPIO

import driver


class MAX6675(driver.TemperatureDriver):
    """
    Python driver for [MAX6675 Cold-Junction Compensated Thermocouple-to-Digital Converter](http://www.adafruit.com/datasheets/MAX6675.pdf)
     Requires:
     - The [GPIO Library](https://code.google.com/p/raspberry-gpio-python/) (Already on most Raspberry Pi OS builds)
     - A [Raspberry Pi](http://www.raspberrypi.org/)

    """
    def __init__(self, options):
        super().__init__(options)
        """
        Initialize Soft (Bitbang) SPI bus
        :param options: options to configure the driver
        option keys:
        - cs_pin:    Chip Select (CS) / Slave Select (SS) pin (Any GPIO)
        - clock_pin: Clock (SCLK / SCK) pin (Any GPIO)
        - data_pin:  Data input (SO / MOSI) pin (Any GPIO)
        - pin_mode:  (optional) pin numbering method as per RPi.GPIO library (GPIO.BCM (default) | GPIO.BOARD)
        """

        self.cs_pin = options['cs_pin']
        self.clock_pin = options['clock_pin']
        self.data_pin = options['data_pin']
        self.data = None
        self.board = GPIO.BCM if options['pin_mode'] is 'BCM' else GPIO.BOARD

        # Initialize needed GPIO
        GPIO.setmode(self.board)
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.setup(self.clock_pin, GPIO.OUT)
        GPIO.setup(self.data_pin, GPIO.IN)

        # Pull chip select high to make chip inactive
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def get_temperature(self, channel: int = 0) -> float:
        self.read()
        self.check_errors()

        data_16 = self.data
        # Remove bits D0-3
        tc_data = ((data_16 >> 3) & 0xFFF)
        # 12-bit resolution
        return tc_data * 0.25

    def read(self):
        """
        Reads 16 bits of the SPI bus & stores as an integer in self.data.
        """
        bytesin = 0
        # Select the chip
        GPIO.output(self.cs_pin, GPIO.LOW)
        # Read in 16 bits
        for i in range(16):
            GPIO.output(self.clock_pin, GPIO.LOW)
            time.sleep(0.001)
            bytesin = bytesin << 1
            if (GPIO.input(self.data_pin)):
                bytesin = bytesin | 1
            GPIO.output(self.clock_pin, GPIO.HIGH)
        time.sleep(0.001)
        # Unselect the chip
        GPIO.output(self.cs_pin, GPIO.HIGH)
        # Save data
        self.data = bytesin

    def check_errors(self, data_16 = None):
        """Checks errors on bit D2"""
        if data_16 is None:
            data_16 = self.data
        no_connection = (data_16 & 0x4) != 0       # tc input bit, D2

        if no_connection:
            raise driver.DriverError(driver.ERROR_NO_CONNECTION, "Thermocouple not connected.")  # open thermocouple

    def cleanup(self):
        """Selective GPIO cleanup"""
        GPIO.setup(self.cs_pin, GPIO.IN)
        GPIO.setup(self.clock_pin, GPIO.IN)


if __name__ == "__main__":

    # default example
    cs_pin = 24
    clock_pin = 23
    data_pin = 22
    thermocouple = MAX6675({'cs_pin': cs_pin, 'clock_pin': clock_pin, 'data_pin': data_pin, 'pin_mode': 'BCM'})
    running = True
    while running:
        try:            
            try:
                tc = thermocouple.get()        
            except driver.DriverError as e:
                tc = "Error: " + e.value
                running = False
                print("tc: {}".format(tc))
            time.sleep(1)
        except KeyboardInterrupt:
            running = False
    thermocouple.cleanup()
