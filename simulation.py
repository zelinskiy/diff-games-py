import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.integrate import odeint

from enum import Enum

from vehicles.simple import AerialVehicleType

class SimulationParams:
    def __init__(self):
        self.N = None

        # self.r_e = None
        # self.r_p = None

        self.q_e = None
        self.q_p = None

        self.k_pf = None
        self.k_ef = None

        self.t_0 = None
        self.t_f = None

        self.d = None
        self.finish_at_d = False

    def set_default(self, N=3):
        self.N = N

        # self.r_e = 2
        # self.r_p = 1

        self.q_e = 2
        self.q_p = 1

        self.k_pf = 10
        self.k_ef = 5

        self.t_0 = 0
        self.t_f = 4

        self.d = 0.1
        self.finish_at_d = False

class SimulationState(Enum):
    INIT = 1
    BUILT = 2
    FINISHED = 1

class Simulation:
    def __init__(self,
            params,
            evader,
            pursuers):
        self.N = params.N

        self.r_e = 1 / evader.max_xyspeed
        self.r_p = 1 / pursuers[0].max_xyspeed

        self.q_e = params.q_e
        self.q_p = params.q_p

        self.k_pf = params.k_pf
        self.k_ef = params.k_ef

        self.t_0 = params.t_0
        self.t_f = params.t_f

        self.d = params.d
        self.finish_at_d = params.finish_at_d

        self.t = np.linspace(self.t_f, self.t_0)

        self.evader = evader
        self.pursuers = pursuers

        self.state = SimulationState.INIT

    def model(self, K, t):
        k_p = K[0]
        k_e = K[1]

        k_p_ = ((-1) * self.q_p
            - (2 / (self.N * self.r_e)) * k_p * k_e
            + (1 / self.r_p) * k_p * k_p)

        k_e_ = ((-1) * self.q_e
            + (1 / self.r_p) * k_p * k_e
            - (1 / (self.N * self.r_e)) * k_e * k_e)

        return np.array([k_p_, k_e_])

    def build(self):
        if(self.state != SimulationState.INIT):
            raise Exception("Simulation not initialized")
        K_f = np.array([self.k_pf, self.k_ef])
        solution = odeint(self.model, K_f, self.t)
        solution = np.flip(solution)
        self.K_p = solution[:, 0]
        self.K_e = solution[:, 1]
        self.state = SimulationState.BUILT


    def run(self):
        if(self.state != SimulationState.BUILT):
            raise Exception("Simulation not built")
        i = 0
        for t_ in self.t:
            x = []
            for pursuer in self.pursuers:
                x.append(pursuer.pos)
                z = pursuer.pos - self.evader.pos
                if(self.finish_at_d and np.all(np.abs(z) < self.d)):
                    print("Cathced by {} at t={}".format(pursuer.name, t_))
                    self.state = SimulationState.FINISHED
                    return
                pcontrol = ((-1) * self.K_p[i] / self.r_p) * z
                pursuer.move(pcontrol)

            z = np.array(x) - np.kron(np.full((self.N, 1), 1), self.evader.pos)
            econtrol = ((-1) / (self.N * self.r_e)) * self.K_e[i] * z
            econtrol = econtrol.mean(axis=0)
            self.evader.move(econtrol)
            i += 1
        self.state = SimulationState.FINISHED

    def show(self, show_ti=True):
        if(self.state != SimulationState.FINISHED):
            raise Exception("Simulation not finished")
        objects = [self.evader]
        for p in self.pursuers:
            objects.append(p)
        if(self.evader.type == AerialVehicleType.D3):
            ax = plt.axes(projection='3d')
        for i, o in enumerate(objects):
            color = 'g' if i == 0 else 'r'

            if(o.type == AerialVehicleType.D2):
                xs, ys = np.array(o.pos_log).T
                plt.plot(xs, ys, c=color, marker = "x",)
            elif(o.type == AerialVehicleType.D3):
                xs, ys, zs = np.array(o.pos_log).T
                ax.plot3D(xs, ys, zs, c=color, marker = "x")

            if(show_ti):
                for i, p in enumerate(o.pos_log):
                    t_ = self.t[i-1]
                    plt.annotate(i, (xs[i], ys[i]))
        plt.show()
