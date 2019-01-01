#!/usr/bin/env python3

import time

import driver


class temperature_simulator(driver.TemperatureDriver):
    def __init__(self, options: dict):
        super().__init__(options)
        self.sleep_time = options['simulation']['step_time']
        self.t_env = options['simulation']['t_env']
        self.c_heat = options['simulation']['c_heat']
        self.c_oven = options['simulation']['c_oven']
        self.p_heat = options['simulation']['p_heat']
        self.r_o_nocool = options['simulation']['R_o_nocool']
        self.r_o_cool = options['simulation']['R_o_cool']
        self.r_ho_noair = options['simulation']['R_ho_noair']
        self.r_ho_air = options['simulation']['R_ho_air']
        self.oven = options['oven']

    def get_temperature(self, channel: int = 0) -> float:
        pass

    def cleanup(self) -> None:
        pass

    def run(self):
        t = self.t_env  # deg C  temp in oven
        t_h = t    # deg C temp of heat element
        while True:
            # heating energy
            q_h = self.p_heat * self.time_step * self.oven.heat

            # temperature change of heat element by heating
            t_h += q_h / self.c_heat

            if self.oven.air:
                r_ho = self.r_ho_air
            else:
                r_ho = self.r_ho_noair

            # energy flux heat_el -> oven
            p_ho = (t_h - t) / r_ho

            # temperature change of oven and heat el
            t += p_ho * self.time_step / self.c_oven
            t_h -= p_ho * self.time_step / self.c_heat

            # energy flux oven -> env
            if self.oven.cool:
                p_env = (t - self.t_env) / self.r_o_cool
            else:
                p_env = (t - self.t_env) / self.r_o_nocool

            # temperature change of oven by cooling to env
            t -= p_env * self.time_step / self.c_oven
            self.log.debug("energy sim: -> %dW heater: %.0f -> %dW oven: %.0f -> %dW env"
                           % (int(self.p_heat * self.oven.heat), t_h, int(p_ho), t, int(p_env)))
            self.temperatures[0] = t

            time.sleep(self.sleep_time)
