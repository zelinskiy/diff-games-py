from vehicles.simple import SimpleAerialVehicle
from simulation import SimulationParams, Simulation
from vehicles.drone import Drone

import numpy as np

import sys

# python2 sim_vehicle.py -v ArduCopter --console --map -w -n 3


params = SimulationParams()
params.set_default(N=2)

params.t_f = 120
params.npoints = 10

params.d = 0.1
params.finish_at_d = True

max_xyspeed = 10


pursuer1 = SimpleAerialVehicle(name="p1", init_pos=np.array([0, 0], dtype='float64'), max_xyspeed=max_xyspeed)
pursuer2 = SimpleAerialVehicle(name="p2", init_pos=np.array([20, 0], dtype='float64'), max_xyspeed=max_xyspeed)

evader = SimpleAerialVehicle(name="e", init_pos=np.array([10, 10], dtype='float64'), max_xyspeed=max_xyspeed * 0.33)

simulation = Simulation(params, evader, [pursuer1, pursuer2])


simulation.build()
simulation.run()
simulation.show(show_ti=True)

sys.exit()

p1_drone = Drone("tcp:127.0.0.1:5763")
p2_drone = Drone("tcp:127.0.0.1:5773")
e_drone = Drone("tcp:127.0.0.1:5783")

gps_coeff = 0.0001

drones = [p1_drone, p2_drone, e_drone]
pos_logs = [pursuer1.pos_log, pursuer2.pos_log, evader.pos_log]

for i, drone in enumerate(drones):
    pos_log = pos_logs[i]
    wps = []
    init_pos = drone.getPosition()[:2]
    for pos in pos_log:
        wp = init_pos + pos * gps_coeff
        wps.append(wp)
    drone.sendWaypoints(wps)

# for drone in drones:
#     drone.startMission()
