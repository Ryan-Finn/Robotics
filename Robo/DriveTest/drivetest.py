import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import math

####################################
#  Build Spline
#  Set ds smaller for less drift, larger for
#    smaller data sets.   0.001 is close and
#    larger than 0.005 is off.
####################################

x = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
y = np.array([0, 2, 4, 3, 2, 1, 2, 4, 5, 7])
tck, u = interpolate.splprep([x, y], s=0.0)
ds = 0.001
t = np.arange(0, 1.01, ds)
out = interpolate.splev(t, tck)
outd = interpolate.splev(t, tck, der=1)
outdd = interpolate.splev(t, tck, der=2)

####################################
#  Plot Spline
####################################

plt.figure()
plt.plot(x, y, 'gs', out[0], out[1], 'b')
plt.legend(['Data', 'Cubic Spline'])
plt.title('Multipoint Spline')
plt.show()

####################################
#  Plug spline points into DD IK
####################################

r = 1.0
L = 4.0
v = np.sqrt(outd[0] * outd[0] + outd[1] * outd[1])
kappa = (outd[0] * outdd[1] - outd[1] * outdd[0]) / (v * v * v)
dotphi1 = (v / r) * (kappa * L + 1)
dotphi2 = (v / r) * (-kappa * L + 1)

####################################
#  Plot wheel speeds as sanity check
####################################

plt.figure()
plt.plot(t, dotphi1, 'b', t, dotphi2, 'r')
plt.legend(['Right Wheel', 'Left Wheel'])
plt.title('Wheel Speeds')
plt.show()

####################################
#  Plug IK data into DD simulation
####################################


N = t.size
one = np.ones(N)
xp = np.zeros(N)
yp = np.zeros(N)
th = np.zeros(N)

xd0 = outd[0][0]
yd0 = outd[1][0]
xp[0] = out[0][0]
yp[0] = out[1][0]
th[0] = math.atan2(yd0, xd0)

for i in range(N - 1):
	xp[i + 1] = xp[i] + (r * ds / 2.0) * (dotphi1[i] + dotphi2[i]) * math.cos(th[i])
	yp[i + 1] = yp[i] + (r * ds / 2.0) * (dotphi1[i] + dotphi2[i]) * math.sin(th[i])
	th[i + 1] = th[i] + (r * ds / (2.0 * L)) * (dotphi1[i] - dotphi2[i])

####################################
#  Plot the robot path and spline
####################################


plt.figure()
plt.plot(x, y, 'gs', out[0], out[1], 'b', xp, yp, 'r')
plt.legend(['Data', 'Interpolant', 'Path'], loc='best')
plt.title('Robot Path')
plt.savefig("RobotPath.pdf", format="pdf")
plt.show()
