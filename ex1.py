from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation

import numpy as np

# Basic example

params = SimulationParams()
params.set_default()


pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([-3, 0], dtype='float64'))
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([3, 0], dtype='float64'))
pursuer3 = SimpleAerialVehicle(name="p3", init_pos=np.array([4, 1], dtype='float64'))

evader = SimpleAerialVehicle(name="e", init_pos=np.array([2, 2], dtype='float64'), max_xyspeed=0.8,)

simulation = Simulation(params, evader, [pursuer1, pursuer2, pursuer3])


simulation.build()
simulation.run()
simulation.show(show_ti=False)
