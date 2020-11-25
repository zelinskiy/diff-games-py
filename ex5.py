from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation

import numpy as np

# To be transformed as SITL simulation

# TODO: add global coefficient

params = SimulationParams()
params.set_default()

params.t_f = 120
params.npoints = 10

params.d = 1
params.finish_at_d = True

max_xyspeed=10


pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([-30, 0], dtype='float64'), max_xyspeed=max_xyspeed)
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([30, 0], dtype='float64'), max_xyspeed=max_xyspeed)
pursuer3 = SimpleAerialVehicle(name="p3", init_pos=np.array([40, 10], dtype='float64'), max_xyspeed=max_xyspeed)

evader = SimpleAerialVehicle(name="e", init_pos=np.array([20, 20], dtype='float64'), max_xyspeed=3)

simulation = Simulation(params, evader, [pursuer1, pursuer2, pursuer3])


simulation.build()
simulation.run()
simulation.show(show_ti=True)
