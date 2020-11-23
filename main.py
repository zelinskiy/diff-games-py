import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

import sys

N = 3
M = 2

x = np.array([[-3, 0], [3, 0], [4, 1]], dtype='float64')

y = np.array([2, 2], dtype='float64')

r_e = 2
r_p = 1

q_e = 2
q_p = 1

k_pf = 10
k_ef = 5

t_0 = 0
t_f = 4

def col1(n):
    return np.full((n, 1), 1)

def model(K, t):
    k_p = K[0]
    k_e = K[1]

    k_p_ = ((-1) * q_p
        - (2 / (N * r_e)) * k_p * k_e
        + (1 / r_p) * k_p * k_p)

    k_e_ = ((-1) * q_e
        + (1 / r_p) * k_p * k_e
        - (1 / (N * r_e)) * k_e * k_e)

    return np.array([k_p_, k_e_])

K_f = np.array([k_pf, k_ef])

t = np.linspace(t_f, t_0)

res = odeint(model, K_f, t)

K_p = np.flip(res)[:, 0]
K_e = np.flip(res)[:, 1]

# sys.exit()

pos_log = []

y_log = []
x0_log = []

i = 0
for t_ in t:
    x0_log.append(np.copy(x[0]))
    y_log.append(np.copy(y))

    z = x - np.kron(col1(N), y)
    up = ((-1) * K_p[i] / r_p) * z
    ue = ((-1) / (N * r_e)) * K_e[i] * z

    ue_x = np.average(ue[:, 0])
    ue_y = np.average(ue[:, 1])
    ue_avg = np.array([ue_x, ue_y])

    y += ue_avg
    x += up

    i += 1

xs, ys = np.array(y_log).T
plt.scatter(xs, ys, c='r')

for i, t_ in enumerate(t):
    plt.annotate(i, (xs[i], ys[i]))

xs, ys = np.array(x0_log).T
plt.scatter(xs, ys, c='g')

for i, t_ in enumerate(t):
    plt.annotate(i, (xs[i], ys[i]))

plt.show()


# plt.plot(t, res)
# plt.xlabel('time')
# plt.ylabel('res')
# plt.show()

'''
def model(y, t):
    k = 0.3
    dydt = -k * y
    return dydt


y0 = 5
t = np.linspace(0, 20)

y = odeint(model, y0, t)


plt.plot(t,y)
plt.xlabel('time')
plt.ylabel('y(t)')
plt.show()
'''
