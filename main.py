import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

import sys

from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation


params = SimulationParams()

params.N = 3

params.r_e = 2
params.r_p = 1

params.q_e = 2
params.q_p = 1

params.k_pf = 10
params.k_ef = 5

params.t_0 = 0
params.t_f = 4

params.d = 0.1
params.finish_at_d = True

pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([-3, 0], dtype='float64'))
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([3, 0], dtype='float64'))
pursuer3 = SimpleAerialVehicle(name="p3", init_pos=np.array([4, 1], dtype='float64'))

evader = SimpleAerialVehicle(name="e", init_pos=np.array([2, 2], dtype='float64'), max_xyspeed=0.8,)

simulation = Simulation(params, evader, [pursuer1, pursuer2, pursuer3])


simulation.build()
simulation.run()
simulation.show()
