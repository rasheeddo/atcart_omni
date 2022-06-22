
import numpy as np

off_ang = np.radians(14.48)
off_ang2 = np.radians(75.52)

AB = 61.0
BC = 180.0
CD = 55.0
AD = 180.0

# theta = np.radians(30)
# phi = np.pi/2.0 - theta

# L = np.sqrt(AB**2 + AD**2 - 2.0*AB*AD*np.cos(phi))

# alpha = np.arcsin((AB/L)*np.sin(phi))

# beta = np.arccos(-(BC**2 - L**2 - CD**2)/(2.0*L*CD))

# gamma_ = np.pi - alpha - beta

# gamma = np.pi/2.0 - gamma_

# print("L: {:.2f} | alp: {:.2f} | beta: {:.2f} | gamma_: {:.2f} | gamma: {:.2f}".format(\
# 	L, np.degrees(alpha), np.degrees(beta), np.degrees(gamma_), np.degrees(gamma)))


gamma = np.radians(90.0)
mu = np.pi/2.0 - gamma

beta = np.pi - mu - off_ang

L = np.sqrt(CD**2 + AD**2 - (2*CD*AD*np.cos(beta)))

alpha = np.arcsin((CD/L)*np.sin(beta))

phi = np.arccos(-((BC**2 - L**2 - AB**2)/(2*L*AB)))

theta = np.pi - off_ang2 - alpha - phi

print("mu: {:.2f} beta: {:.2f} L: {:.2f} phi: {:.2f} theta: {:.2f}".format(\
	np.degrees(mu), np.degrees(beta), L, np.degrees(phi), np.degrees(theta)))