from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation

import numpy as np


params = SimulationParams()
params.set_default(N=2)
params.finish_at_d = True
params.d = 0.1
params.t_f = 20

pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([10, 0, 0], dtype='float64'))
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([0, 0, 0], dtype='float64'))

evader = SimpleAerialVehicle(name="e", init_pos=np.array([5, 5, 3], dtype='float64'), max_xyspeed=0.2, max_zspeed=0.1)

simulation = Simulation(params, evader, [pursuer1, pursuer2])


simulation.build()
simulation.run()
simulation.show(show_ti=False)
