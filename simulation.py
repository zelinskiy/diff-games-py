import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint

from enum import Enum

class SimulationParams:
    def __init__(self):
        self.N = None

        self.r_e = None
        self.r_p = None

        self.q_e = None
        self.q_p = None

        self.k_pf = None
        self.k_ef = None

        self.t_0 = None
        self.t_f = None

        self.d = None
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

        self.r_e = params.r_e
        self.r_p = params.r_p

        self.q_e = params.q_e
        self.q_p = params.q_p

        self.k_pf = params.k_pf
        self.k_ef = params.k_ef

        self.t_0 = params.t_0
        self.t_f = params.t_f

        self.d = params.d

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
            econtrol = np.array([np.average(econtrol[:, 0]), np.average(econtrol[:, 1])])
            self.evader.move(econtrol)
            i += 1
        self.state = SimulationState.FINISHED

    def show(self):
        if(self.state != SimulationState.FINISHED):
            raise Exception("Simulation not finished")
        objects = [self.evader]
        for p in self.pursuers:
            objects.append(p)
        for i, o in enumerate(objects):
            color = 'g' if i == 0 else 'r'

            xs, ys = np.array(o.pos_log).T
            plt.scatter(xs, ys, c=color)

            for i, p in enumerate(o.pos_log):
                t_ = self.t[i-1]
                plt.annotate(i, (xs[i], ys[i]))
        plt.show()
