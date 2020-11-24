import random
from enum import Enum

import numpy as np

class AerialVehicleType(Enum):
    D2 = 2
    D3 = 3


class SimpleAerialVehicle:

    def __init__(self,
            init_pos=np.array([0, 0], dtype='float64'),
            type=AerialVehicleType.D2,
            name="Unnamed",
            max_xyspeed=1,
            max_zspeed=0.5,
            speed_coeff=0.15,
            min_noise=0,
            max_noise=0):
        self.pos = np.copy(init_pos)
        self.max_xyspeed = max_xyspeed
        self.max_zspeed = max_zspeed
        self.min_noise = min_noise
        self.max_noise = max_noise
        self.type = type
        self.pos_log = [init_pos]
        self.speed_coeff = speed_coeff
        self.name = name

    def get_noise(self):
        if(self.max_noise != 0):
            sign = random.choice([-1,1])
            value = random.uniform(self.min_noise, self.max_noise)
            return np.full(self.pos.shape, sign * value)
        else:
            return np.full(self.pos.shape, 0)

    def move(self, ctrl):
        control = np.copy(ctrl)
        if(control[0] > self.max_xyspeed):
            control[0] = self.max_xyspeed
        if(control[1] > self.max_xyspeed):
            control[1] = self.max_xyspeed
        if(self.type is AerialVehicleType.D3 and control[2] > self.max_zspeed):
            control[2] = self.max_zspeed

        self.pos += control * self.speed_coeff
        self.pos_log.append(np.copy(self.pos))
        return self.pos
