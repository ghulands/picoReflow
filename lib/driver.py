#!/usr/bin/env python3

import abc
import logging
import threading
import time

"""
Common Error Codes for Thermocouples
"""
ERROR_UNKNOWN = -1
ERROR_NO_CONNECTION = 1
ERROR_SHORT_TO_GROUND = 2
ERROR_SHORT_TO_VCC = 3


class DriverError(IOError):
    def __init__(self, code: int, message: str):
        self.code = code
        self.message = message


class TemperatureDriver(threading.Thread, metaclass=abc.ABCMeta):
    def __init__(self, options: dict):
        super().__init__()
        self.options = options
        self.daemon = True
        self.temperatures = [0.0] * self.number_of_channels()
        self.log = logging.getLogger('TemperatureDriver')
        self.lock = threading.RLock()
        if 'time_step' in options:
            self.time_step = options['time_step']
        else:
            self.time_step = 0.5

    def get(self, channel: int = 0) -> float:
        with self.lock:
            temperature = self.temperatures[channel]
        return temperature

    def number_of_channels(self) -> int:
        return 1

    @abc.abstractmethod
    def get_temperature(self, channel: int = 0) -> float:
        """
        Get the temperature from the device
        :param channel: the channel to read from.
        :return: the temperature in Celcius.
        :except: DriverError
        """
        raise NotImplementedError('concrete drivers need to implement')

    @abc.abstractmethod
    def cleanup(self) -> None:
        """
        Called when tearing everything down.
        :return:
        """
        raise NotImplementedError('concrete drivers need to implement')

    def run(self):
        while True:
            with self.lock:
                try:
                    for i in range(0, self.number_of_channels()):
                        self.temperatures[i] = self.get_temperature(i)
                except DriverError:
                    self.log.exception("Problem reading temp")
            time.sleep(self.time_step)


class DoorDriver(threading.Thread, metaclass=abc.ABCMeta):
    def __init__(self, options: dict):
        super().__init__(options)
        self.options = options

    @abc.abstractmethod
    def open_door(self):
        raise NotImplementedError('concrete drivers need to implement')

    @abc.abstractmethod
    def close_door(self):
        raise NotImplementedError('concrete drivers need to implement')

    @abc.abstractmethod
    def is_open(self):
        raise NotImplementedError('concrete drivers need to implement')


class FanDriver(threading.Thread, metaclass=abc.ABCMeta):
    def __init__(self, options: dict):
        super().__init__()
        self.options = options

    @abc.abstractmethod
    def set_speed(self, speed: float) -> None:
        """
        Set the speed of the fan.
        :param speed: 0-1 as a percentage
        :return:
        """
        raise NotImplementedError('concrete drivers need to implement')


class HeaterDriver(threading.Thread, metaclass=abc.ABCMeta):
    def __init__(self, options: dict):
        super().__init__()
        self.options = options

    @abc.abstractmethod
    def turn_on(self) -> None:
        """
        Turn on the heater
        :return:
        """
        raise NotImplementedError('concrete drivers need to implement')

    @abc.abstractmethod
    def turn_off(self) -> None:
        """
        Turn off the heater
        :return:
        """
        raise NotImplementedError('concrete drivers need to implement')

