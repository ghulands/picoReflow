#!/usr/bin/env python3

import threading
import datetime
import logging

import RPi.GPIO as GPIO

log = logging.getLogger('gpiodriver')

class gpio(threading.Thread):
    def __init__(self, options: dict):
        super().__init__()
        self.options = options

        self.pin = options['bcm_pin']

        GPIO.setmode(GPIO.BCM)

        self.gpio_type = options['type']
        if self.gpio_type == 'switch':
            self.last_switched_on = datetime.datetime.now()
            self.max_switching_frequency = options['hz_limit']
            self.inverting = options['invert'] if 'invert' in options else False
            GPIO.setup(self.pin, GPIO.OUT, initial=1 if self.inverting else 0)
            self.turn_off()
        elif self.gpio_type == 'pwm':
            GPIO.setup(self.pin, GPIO.OUT)
            self.pwm_min = options['pwm_min']
            self.pwm_max = options['pwm_max']
            self.pwm = GPIO.PWM(self.pin, options['pwm_freq'])
            self.pwm.start(self.pwm_min)
            self.turn_off()
        elif self.gpio_type == 'input':
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP if options['pud'] == "up" else GPIO.PUD_DOWN)
            GPIO.add_event_detect(self.pin,
                                  GPIO.FALLING if options['edge'] == 'falling' else GPIO.RISING,
                                  callback=self.input_occurred,
                                  bouncetime=options['debounce_ms'])
        self.on = False

    def input_occurred(self, channel):
        self.on = GPIO.input(self.pin)

    def set_speed(self, speed: float) -> None:
        if speed == 0.0:
            self.turn_off()
        if speed == 1.0:
            self.turn_on()
        if self.gpio_type == 'pwm':
            scaled_dutycycle = (self.pwm_max - self.pwm_min) * speed
            self.pwm.ChangeDutyCycle(scaled_dutycycle)
            self.on = True

    def turn_on(self) -> None:
        self.on = True
        if self.gpio_type == 'switch':
            now = datetime.datetime.now()
            if (now - self.last_switched_on).total_seconds() < 1.0 / self.max_switching_frequency:
                log.debug('switching frequency too fast. Ignoring commanded switch on.')
                return
            if self.inverting:
                GPIO.output(self.pin, GPIO.LOW)
            else:
                GPIO.output(self.pin, GPIO.HIGH)
            self.last_switched_on = now
        elif self.gpio_type == 'pwm':
            self.pwm.ChangeDutyCycle(self.pwm_max)

    def turn_off(self) -> None:
        self.on = False
        if self.gpio_type == 'switch':
            if self.inverting:
                GPIO.output(self.pin, GPIO.HIGH)
            else:
                GPIO.output(self.pin, GPIO.LOW)
        elif self.gpio_type == 'pwm':
            self.pwm.ChangeDutyCycle(self.pwm_min)

    def is_on(self) -> bool:
        return self.on
