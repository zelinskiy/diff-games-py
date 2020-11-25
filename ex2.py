from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation

import numpy as np

# Example with 5 pursuers

params = SimulationParams()
params.set_default(N=5)
params.finish_at_d = True
params.t_f = 10


pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([20, 0], dtype='float64'))
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([0, 0], dtype='float64'))
pursuer3 = SimpleAerialVehicle(name="p3", init_pos=np.array([0, 20], dtype='float64'))
pursuer4= SimpleAerialVehicle(name="p3", init_pos=np.array([20, 20], dtype='float64'))
pursuer5 = SimpleAerialVehicle(name="p3", init_pos=np.array([0, 10], dtype='float64'))

evader = SimpleAerialVehicle(name="e", init_pos=np.array([10, 10], dtype='float64'), max_xyspeed=0.62,)

simulation = Simulation(params, evader, [pursuer1, pursuer2, pursuer3, pursuer4, pursuer5])


simulation.build()
simulation.run()
simulation.show(show_ti=False)
