#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import sys


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h


prefix = "exp_adaptability/default/"
# prefix = "exp_adaptability/robot_failure/"
# prefix = "exp_adaptability/goal_change/"
experiments = ["exp_000.npy", "exp_001.npy", "exp_002.npy", "exp_003.npy", "exp_004.npy", "exp_005.npy", "exp_006.npy", "exp_007.npy", "exp_008.npy", "exp_009.npy"]
data_vel = []
data_time = []
data_dist = []
end = 440000
for experiment in experiments:
    print("Reading file: " + prefix+experiment)
    d = np.load(prefix+experiment)
    print(d.shape)
    data_time.append(d[:end, 0])
    data_dist.append(d[:end, 1])
    data_vel.append(d[:end, 2])

data_time = np.transpose(np.array(data_time))
data_dist = np.transpose(np.array(data_dist))
data_vel = np.transpose(np.array(data_vel))
print(data_time.shape)
print(data_dist.shape)
print(data_vel.shape)

# data = []
# print("processing...")
# for i in range(0, end):
#     m_dist, l_dist, u_dist = mean_confidence_interval(data_dist[i, :])
#     m_vel, l_vel, u_vel = mean_confidence_interval(data_vel[i, :])
#     data.append([data_time[i, 0], m_dist, l_dist, u_dist, m_vel, l_vel, u_vel])
# data = np.array(data)
# print("saving...")
# np.save("exp_adaptability/default.npy", data)
# exit()


data_dft = np.load("exp_adaptability/default.npy")
data_rfai = np.load("exp_adaptability/robot_failure.npy")
data_gch = np.load("exp_adaptability/goal_change.npy")
datas = [data_dft, data_rfai, data_gch]

# Ploting data
# fig = plt.figure(dpi=200)
# ax = fig.add_subplot(111)
# color = 'tab:red'
# ax.plot(data[:, 0], data[:, 5]*100, "--", color=color, linewidth=0.4)
# ax.plot(data[:, 0], data[:, 6]*100, "--", color=color, linewidth=0.4)
# ax.plot(data[:, 0], data[:, 4]*100, label='object velocity', color=color)
# ax.set_xlabel('time (seconds)')
# ax.set_ylabel('object velocity (cm/s)', color=color)
# ax.tick_params(axis='y', labelcolor=color)
# ax.set_ylim([0, 3.2])


# ax2 = ax.twinx()
# color = 'tab:blue'
# ax2.set_ylabel('distance to goal (m)', color=color)
# ax2.plot(data[:, 0], data[:, 2], "--", color=color, linewidth=0.2)
# ax2.plot(data[:, 0], data[:, 3], "--", color=color, linewidth=0.2)
# ax2.plot(data[:, 0], data[:, 1], label='distance to goal', color=color)
# ax2.tick_params(axis='y', labelcolor=color)
# ax2.set_ylim([0, 2.2])

# ax.set_title("Convergence Analyses")

# # plt.savefig(figfile, dpi=200)
# plt.show()


fig, axs = plt.subplots(3, sharex=True, sharey=False,
                        gridspec_kw={'hspace': 0.1})

for i in range(0, 3):
    axs[i].set_rasterized(True)
    ax = axs[i]
    color = 'tab:red'
    data = datas[i]
    ax.plot(data[:, 0], data[:, 5]*100, "--",
            color=color, linewidth=0.2, alpha=0.1)
    ax.plot(data[:, 0], data[:, 6]*100, "--",
            color=color, linewidth=0.2, alpha=0.1)
    ax.plot(data[:, 0], data[:, 4]*100, label='object velocity', color=color)
    if i == 1:
        ax.set_ylabel('Object velocity (cm/s)', color=color, fontsize=14)
    ax.tick_params(axis='y', labelcolor=color)
    ax.set_ylim([0, 3.2])

    ax2 = ax.twinx()
    ax2.set_rasterized(True)
    color = 'tab:blue'
    if i == 1:
        ax2.set_ylabel('Distance to goal (m)', color=color, fontsize=14)
    ax2.plot(data[:, 0], data[:, 2], "--",
             color=color, linewidth=0.8, alpha=1.0)
    ax2.plot(data[:, 0], data[:, 3], "--",
             color=color, linewidth=0.8, alpha=1.0)

    ax2.plot(data[:, 0], data[:, 1], label='distance to goal', color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    ax2.set_ylim([0, 2.2])

axs[0].set_title("Convergence Analyses", fontsize=16)
axs[2].set_xlabel('Time (seconds)', fontsize=14)


plt.savefig("adaptability.pdf", dpi=200)
plt.show()
