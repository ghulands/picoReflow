#!/usr/bin/env python3

import driver
import json
import serial
import time


class max31855json(driver.TemperatureDriver):
    def __init__(self, options: dict):
        super().__init__(options)
        self.last_response = None
        try:
            self.serial_port = serial.Serial(port=options['serial_port'], baudrate=115200)
        except Exception:
            pass

    def number_of_channels(self) -> int:
        return self.options['channels']

    def get_temperature(self, channel: int = 0) -> float:
        pass  # we fill the self.temperatures directly in the overridden run method

    def run(self):
        # configure the device

        while True:
            try:
                json_str = self.serial_port.readline()
                if not str(json_str).startswith('{'):
                    continue
                self.log.debug('received json string: %s', json_str)
                decoded = json.loads(json_str, encoding='ascii')
                self.last_response = decoded
                with self.lock:
                    for sensor in decoded.temperature:
                        self.temperatures[sensor.channel] = sensor.value
            except driver.DriverError:
                self.log.exception("Problem reading temp")
            time.sleep(self.time_step)

    def cleanup(self):
        self.serial_port.close()
        self.serial_port = None
