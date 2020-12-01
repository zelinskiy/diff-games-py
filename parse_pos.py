import matplotlib.pyplot as plt
import numpy as np
import numpy.polynomial.polynomial as poly
import numpy.polynomial.chebyshev as cheb
import numpy.polynomial.hermite as herm
import numpy.polynomial.hermite_e as herme
import numpy.polynomial.laguerre as lag
import numpy.polynomial.legendre as leg

# TBD: data cleaning

FILEPATH = "pos.txt"

# For storage
times = []
alts = []
lons = []
lats = []

print("Begin parsing coordinates")
# print("t, lat, lon, alt")
with open(FILEPATH, "r") as file:
    lines = file.readlines()
    t_0 = int(lines[0].split(", ")[1])
    for row_raw in lines:
        row = row_raw.split(", ")
        t_raw = int(row[1])
        t = int((t_raw - t_0) / 100000)
        lat = float(row[2])
        lon = float(row[3])
        alt = float(row[4])
        times.append(t)
        lats.append(lat)
        lons.append(lon)
        alts.append(alt)
        # print("{}, {}, {}, {}".format(t, lat, lon, alt))

print("Parsing done")



def mkseries(xs, ys, p):
    xs_ = np.linspace(xs[0], xs[-1])
    ys_ = p(xs_)
    err = np.sum(p(xs) - ys_)
    print(err)
    return (xs_, ys_)


def polyfit(xs, ys, deg):
    coeffs = poly.polyfit(xs, ys, deg)
    p = poly.Polynomial(coeffs)
    return mkseries(xs, ys, p)

def chebfit(xs, ys, deg):
    coeffs = cheb.chebfit(xs, ys, deg)
    p = cheb.Chebyshev(coeffs)
    return mkseries(xs, ys, p)

def hermfit(xs, ys, deg):
    coeffs = herm.hermfit(xs, ys, deg)
    p = herm.Hermite(coeffs)
    return mkseries(xs, ys, p)

def hermfit2(xs, ys, deg):
    coeffs = herme.hermefit(xs, ys, deg)
    p = herme.HermiteE(coeffs)
    return mkseries(xs, ys, p)

def lagfit(xs, ys, deg):
    coeffs = lag.lagfit(xs, ys, deg)
    p = lag.Laguerre(coeffs)
    return mkseries(xs, ys, p)

def legfit(xs, ys, deg):
    coeffs = leg.legfit(xs, ys, deg)
    p = leg.Legendre(coeffs)
    return mkseries(xs, ys, p)

start = 1030
end = 1080

xs = np.array(lons[start:end])
ys = np.array(lats[start:end])

c_x = xs[int(len(xs) / 2)]
c_y = ys[int(len(ys) / 2)]

for i, x in enumerate(xs):
    xs[i] = x - c_x

for i, y in enumerate(ys):
    ys[i] = y - c_y

DEG = 1

fig, plts = plt.subplots(2, 3)

power_xs, power_ys = polyfit(xs, ys, DEG)
plts[0, 0].plot(xs, ys, "o", power_xs, power_ys)
plts[0, 0].title.set_text('Power')

cheb_xs, cheb_ys = chebfit(xs, ys, DEG)
plts[0, 1].plot(xs, ys, "o", cheb_xs, cheb_ys)
plts[0, 1].title.set_text('Chebyshev')

herm_xs, herm_ys = hermfit(xs, ys, DEG)
plts[0, 2].plot(xs, ys, "o", herm_xs, herm_ys)
plts[0, 2].title.set_text('Hermite (1)')

herm2_xs, herm2_ys = hermfit2(xs, ys, DEG)
plts[1, 0].plot(xs, ys, "o", herm2_xs, herm2_ys)
plts[1, 0].title.set_text('Hermite (2)')

lag_xs, lag_ys = lagfit(xs, ys, DEG)
plts[1, 1].plot(xs, ys, "o", lag_xs, lag_ys)
plts[1, 1].title.set_text('Laguerre')

leg_xs, leg_ys = legfit(xs, ys, DEG)
plts[1, 2].plot(xs, ys, "o", leg_xs, leg_ys)
plts[1, 2].title.set_text('Legendre')


plt.show()

print("DONE")
