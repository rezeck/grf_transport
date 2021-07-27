#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import sys
from tqdm import tqdm 


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h


prefix = sys.argv[1]
experiments = []
for i in range(int(sys.argv[2]),int(sys.argv[3])+1):
    experiments.append("exp_{:03d}.npy".format(i))
print(experiments)
data_vel = []
data_time = []
data_dist = []

end = []
print("Cutting data to fit to small one...")
for experiment in experiments:
    d = np.load(prefix+experiment)
    end.append(d.shape[0])
    print(d.shape[0])
end = min(end)
print("Cutted at {}".format(end))


for experiment in tqdm(experiments):
    print("Reading file: " + prefix+experiment)
    d = np.load(prefix+experiment)
    data_time.append(d[:end, 0])
    data_dist.append(d[:end, 1])
    data_vel.append(d[:end, 2])

data_time = np.transpose(np.array(data_time))
data_dist = np.transpose(np.array(data_dist))
data_vel = np.transpose(np.array(data_vel))

data = []
print("Processing stats...")
for i in  tqdm(range(0, end)):
    m_dist, l_dist, u_dist = mean_confidence_interval(data_dist[i, :])
    m_vel, l_vel, u_vel = mean_confidence_interval(data_vel[i, :])
    data.append([data_time[i, 0], m_dist, l_dist, u_dist, m_vel, l_vel, u_vel])
data = np.array(data)
print("saving...")
filename = prefix.replace('/', '.')
if filename[-1] == '.':
    filename += "npy"
else:
    filename += ".npy"
np.save(filename, data)
