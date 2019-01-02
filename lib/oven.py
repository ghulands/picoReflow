#!/usr/bin/env python3

import threading
import time
import datetime
import logging
import json
import importlib.util
import statistics

import driver


log = logging.getLogger(__name__)


class Oven (threading.Thread):
    STATE_IDLE = "IDLE"
    STATE_RUNNING = "RUNNING"

    def __init__(self, config: dict):
        threading.Thread.__init__(self)
        self.config = config
        self.daemon = True
        self.time_step = config['control']['time_step']
        self.profile = None
        self.start_time = 0
        self.runtime = 0
        self.total_time = 0
        self.target = 0
        self.state = Oven.STATE_IDLE
        self.pid = PID(ki=config['control']['pid']['ki'],
                       kd=config['control']['pid']['kd'],
                       kp=config['control']['pid']['kp'])
        self.drivers = {}
        self.chamber_sensors = []
        self.pcb_sensors = []
        self.chamber_fans = []
        self.auxiliary_fans = []
        self.heater_actuators = []
        self.door_actuators = []
        self.door_sensors = []

        self.heat = 0.0
        self.cool = 0.0
        self.air = 0.0

        # instantiate the drivers
        for drv in config['oven']['drivers']:
            driver_class = drv['driver']
            try:
                spec = importlib.util.find_spec(driver_class)
                mod = importlib.import_module(spec.name)
                klass = getattr(mod, driver_class)
                options = drv['driver_options']
                options['oven'] = self
                instance = klass(options)
                instance.start()
                self.drivers[drv['name']] = instance
            except ImportError as e:
                log.error('Unable to load driver %s', driver_class)
                log.exception(e)

        # connect the logical devices to the drivers
        for tmp in config['oven']['temperature']:
            logical_name = tmp['name']
            sensor_type = tmp['type']
            driver_instance = tmp['driver']
            channel = 0
            if 'channel' in tmp:
                channel = tmp['channel']
            if driver_instance not in self.drivers:
                log.error('Unable to find instance of driver %s for sensor %s', driver_instance, logical_name)
            sensor_instance = TemperatureSensor(logical_name, self.drivers[driver_instance], channel)
            if sensor_type == 'chamber':
                self.chamber_sensors.append(sensor_instance)
            elif sensor_type == 'pcb':
                self.pcb_sensors.append(sensor_instance)
            else:
                log.error('Unknown temperature sensor type %s.', sensor_type)

        for fan in config['oven']['fan']:
            logical_name = fan['name']
            driver_instance = fan['driver']
            fan_type = fan['type']
            if driver_instance not in self.drivers:
                log.error('Unable to find instance of driver %s for fan actuator %s', driver_instance, logical_name)
            actuator_instance = FanActuator(self.drivers[driver_instance])
            if fan_type == 'chamber':
                self.chamber_fans.append(actuator_instance)
            elif fan_type == 'aux':
                self.auxiliary_fans.append(actuator_instance)
            else:
                log.error('Unknown fan type %s', fan_type)

        for heater in config['oven']['heater']:
            logical_name = heater['name']
            driver_instance = heater['driver']
            if driver_instance not in self.drivers:
                log.error('Unable to find instance of driver %s for heater actuator %s', driver_instance, logical_name)
            actuator_instance = HeaterActuator(self.drivers[driver_instance])
            self.heater_actuators.append(actuator_instance)

        self.door = self.get_door_state()
        self.reset()
        self.start()

    def reset(self):
        self.profile = None
        self.start_time = 0
        self.runtime = 0
        self.total_time = 0
        self.target = 0
        self.door = self.get_door_state()
        self.state = Oven.STATE_IDLE
        self.set_heat(False)
        self.set_cool(False)
        self.set_air(False)
        self.pid = PID(ki=self.config['control']['pid']['ki'],
                       kd=self.config['control']['pid']['kd'],
                       kp=self.config['control']['pid']['kp'])

    def run_profile(self, profile):
        log.info("Running profile %s" % profile.name)
        self.profile = profile
        self.total_time = profile.get_duration()
        self.state = Oven.STATE_RUNNING
        self.start_time = datetime.datetime.now()
        log.info("Starting")

    def abort_run(self):
        self.reset()

    def get_chamber_temperature(self):
        count = 0
        total = 0
        for x in self.chamber_sensors:
            if x > 0:
                total = total + x
                count = count + 1
        avg = total / count
        return avg

    def chamber_has_variance(self):
        result = False
        if len(self.chamber_sensors) > 1:
            result = statistics.stdev([x.get() for x in self.chamber_sensors]) > 20  # this should come from config
        return result

    def get_pcb_temperature(self):
        count = 0
        total = 0
        for x in self.pcb_sensors:
            if x > 0:
                total = total + x
                count = count + 1
        avg = total / count
        return avg

    def run(self):
        temperature_count = 0
        last_temp = 0
        pid = 0
        while True:
            self.door = self.get_door_state()

            if self.state == Oven.STATE_RUNNING:
                runtime_delta = datetime.datetime.now() - self.start_time
                self.runtime = runtime_delta.total_seconds()
                log.info("running at %.1f deg C (Target: %.1f) , heat %.2f, cool %.2f, air %.2f, door %s (%.1fs/%.0f)"
                         % (self.get_pcb_temperature(),
                            self.target,
                            self.heat,
                            self.cool,
                            self.air,
                            self.door,
                            self.runtime,
                            self.total_time))
                self.target = self.profile.get_target_temperature(self.runtime)
                pid = self.pid.compute(self.target, self.get_pcb_temperature())

                log.info("pid: %.3f" % pid)

                self.set_cool(pid <= -1)
                if pid > 0:
                    # The temp should be changing with the heat on
                    # Count the number of time_steps encountered with no change and the heat on
                    if last_temp == self.get_pcb_temperature():
                        temperature_count += 1
                    else:
                        temperature_count = 0
                    # If the heat is on and nothing is changing, reset
                    # The direction or amount of change does not matter
                    # This prevents runaway in the event of a sensor read failure                   
                    if temperature_count > 20:
                        temperature_count = 0
                        log.info("Error reading sensor, oven temp not responding to heat.")
                        self.reset()
                else:
                    temperature_count = 0
                    
                #Capture the last temperature value.  This must be done before set_heat, since there is a sleep in there now.
                last_temp = self.get_pcb_temperature()
                
                self.set_heat(pid)
                
                #if self.profile.is_rising(self.runtime):
                #    self.set_cool(False)
                #    self.set_heat(self.get_pcb_temperature() < self.target)
                #else:
                #    self.set_heat(False)
                #    self.set_cool(self.get_pcb_temperature() > self.target)

                if self.get_pcb_temperature() > 200:
                    self.set_air(False)
                elif self.get_pcb_temperature() < 180 or self.chamber_has_variance():
                    self.set_air(True)

                if self.runtime >= self.total_time:
                    self.reset()

            if 0 < pid < 1.0:
                time.sleep(self.time_step * (1 - pid))
            else:
                time.sleep(self.time_step)

    def set_heat(self, value):
        if value > 0:
            self.heat = 1.0
            for heater in self.heater_actuators:
                heater.turn_on()
        else:
            self.heat = 0.0
            for heater in self.heater_actuators:
                heater.turn_off()

    def set_cool(self, value):
        if value:
            self.cool = 1.0
            for fan in self.chamber_fans:
                fan.set_speed(1.0)
            for door in self.door_actuators:
                door.open()

        else:
            self.cool = 0.0
            for fan in self.chamber_fans:
                fan.set_speed(0.0)
            for door in self.door_actuators:
                door.close()

    def set_air(self, value):
        if value:
            self.air = 1.0
            for fan in self.chamber_fans:
                fan.set_speed(1.0)
        else:
            self.air = 0.0
            for fan in self.chamber_fans:
                fan.set_speed(0.0)

    def get_state(self):
        chamber_temps = []
        for temp in self.chamber_sensors:
            chamber_temps.append({'name': temp.name, 'celcius': '{0:.2f}'.format(temp.get())})
        pcb_temps = []
        for temp in self.pcb_sensors:
            pcb_temps.append({'name': temp.name, 'celcius': '{0:.2f}'.format(temp.get())})
        state = {
            'runtime': self.runtime,
            'temperature': self.chamber_sensors[0].get(),
            'temperatures': {
                'chamber': chamber_temps,
                'pcb': pcb_temps
            },
            'target': self.target,
            'state': self.state,
            'heat': self.heat,
            'cool': self.cool,
            'air': self.air,
            'totaltime': self.total_time,
            'door': self.door
        }
        return state

    def get_door_state(self):
        for sensor in self.door_sensors:
            if sensor.is_open():
                return "OPEN"

        return "CLOSED"


class TemperatureSensor(object):
    def __init__(self, name: str, driver_instance: driver.TemperatureDriver, channel: int = 0):
        self.name = name
        self.driver_instance = driver_instance
        self.channel = channel

    def get(self):
        return self.driver_instance.get(self.channel)


class FanActuator(object):
    def __init__(self, driver_instance: driver.FanDriver):
        self.driver_instance = driver_instance

    def set_speed(self, speed):
        self.driver_instance.set_speed(speed)


class HeaterActuator(object):
    def __init__(self, driver_instance: driver.HeaterDriver):
        self.driver_instance = driver_instance

    def turn_on(self):
        self.driver_instance.turn_on()

    def turn_off(self):
        self.driver_instance.turn_off()



class Profile():
    def __init__(self, json_data):
        obj = json.loads(json_data)
        self.name = obj["name"]
        self.data = sorted(obj["data"])

    def get_duration(self):
        return max([t for (t, x) in self.data])

    def get_surrounding_points(self, time):
        if time > self.get_duration():
            return None, None

        prev_point = None
        next_point = None

        for i in range(len(self.data)):
            if time < self.data[i][0]:
                prev_point = self.data[i-1]
                next_point = self.data[i]
                break

        return (prev_point, next_point)

    def is_rising(self, time):
        (prev_point, next_point) = self.get_surrounding_points(time)
        if prev_point and next_point:
            return prev_point[1] < next_point[1]
        else:
            return False

    def get_target_temperature(self, time):
        if time > self.get_duration():
            return 0

        (prev_point, next_point) = self.get_surrounding_points(time)

        incl = float(next_point[1] - prev_point[1]) / float(next_point[0] - prev_point[0])
        temp = prev_point[1] + (time - prev_point[0]) * incl
        return temp


class PID(object):
    def __init__(self, ki=1, kp=1, kd=1):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.lastNow = datetime.datetime.now()
        self.iterm = 0
        self.lastErr = 0

    def compute(self, set_point, is_point):
        now = datetime.datetime.now()
        time_delta = (now - self.lastNow).total_seconds()

        error = float(set_point - is_point)
        self.iterm += (error * time_delta * self.ki)
        self.iterm = sorted([-1, self.iterm, 1])[1]
        d_err = (error - self.lastErr) / time_delta

        output = self.kp * error + self.iterm + self.kd * d_err
        output = sorted([-1, output, 1])[1]
        self.lastErr = error
        self.lastNow = now

        return output
