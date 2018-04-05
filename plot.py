#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

# cat out.txt | grep Collision! | awk '{ print $3, $5, $7, $9, $11, $13, $15, $17, $19, $21 }' > out.csv

data_x = np.genfromtxt("q_avoid_diag_x.csv", delimiter=",", usecols=range(19))
data_y = np.genfromtxt("q_avoid_diag_y.csv", delimiter=",", usecols=range(19))
scale = np.sqrt(np.power(data_x, 2) + np.power(data_y, 2))
plt.quiver(data_x / scale, data_y / scale, width = 0.002, minlength=1.5) #, scale = 1000)
plt.show()
exit()

data = np.genfromtxt("q_diag.csv", delimiter=",", usecols=range(74))
data[np.logical_or(data < -1e20, data > 1e20)] = float('nan')
# import pdb; pdb.set_trace()
plt.imshow(data)
plt.show()
exit()

data = np.genfromtxt("out.csv", delimiter=" ", dtype=None)
depth1 = np.array([a[0] for a in data])
rel_vel1 = np.array([a[1] for a in data])
intensity1 = np.array([a[2] for a in data])
depth2 = np.array([a[3] for a in data])
rel_vel2 = np.array([a[4] for a in data])
intensity2 = np.array([a[5] for a in data])
kind1 = np.array([a[6] for a in data])
damage1 = np.array([a[7] for a in data])
kind2 = np.array([a[8] for a in data])
damage2 = np.array([a[9] for a in data])

pbp = np.concatenate((damage1[np.logical_and(kind1 == b"Pedestrian", kind2 == b"Pedestrian")],
                         damage2[np.logical_and(kind1 == b"Pedestrian", kind2 == b"Pedestrian")]))
pbv = np.concatenate((damage1[np.logical_and(kind1 == b"Pedestrian", kind2 == b"Vehicle")],
                         damage2[np.logical_and(kind1 == b"Vehicle", kind2 == b"Pedestrian")]))
pbo = np.concatenate((damage1[np.logical_and(kind1 == b"Pedestrian", kind2 == b"Obstacle")],
                         damage2[np.logical_and(kind1 == b"Obstacle", kind2 == b"Pedestrian")]))
vbp = np.concatenate((damage1[np.logical_and(kind1 == b"Vehicle", kind2 == b"Pedestrian")],
                         damage2[np.logical_and(kind1 == b"Pedestrian", kind2 == b"Vehicle")]))
vbv = np.concatenate((damage1[np.logical_and(kind1 == b"Vehicle", kind2 == b"Vehicle")],
                         damage2[np.logical_and(kind1 == b"Vehicle", kind2 == b"Vehicle")]))
vbo = np.concatenate((damage1[np.logical_and(kind1 == b"Vehicle", kind2 == b"Obstacle")],
                         damage2[np.logical_and(kind1 == b"Obstacle", kind2 == b"Vehicle")]))
plt.hist([pbp, pbv, pbo, vbp, vbv, vbo], label=["P by P", "P by V", "P by O", "V by P", "V by V", "V by O"])
plt.legend()
plt.show()

# import pdb; pdb.set_trace()
