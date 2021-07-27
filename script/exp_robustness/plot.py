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


print(len(sys.argv))
filenames = sys.argv[1:]
print(filenames)
datas = []

for filename in tqdm(filenames):
    print("Reading the file: {}".format(filename))
    datas.append(np.load(filename))



fig, axs = plt.subplots(len(filenames), sharex=True, sharey=False,
                        gridspec_kw={'hspace': 0.1})

for i in range(0, len(filenames)):
    axs[i].set_rasterized(True)
    ax = axs[i]
    color = 'tab:red'
    data = datas[i]
    ax.plot(data[:, 0], data[:, 5], "--",
            color=color, linewidth=0.2, alpha=0.3)
    ax.plot(data[:, 0], data[:, 6], "--",
            color=color, linewidth=0.2, alpha=0.3)
    ax.plot(data[:, 0], data[:, 4], label='object velocity', color=color)
    if i == 1:
        ax.set_ylabel('Object velocity (cm/s)', color=color, fontsize=14)
    ax.tick_params(axis='y', labelcolor=color)
    ax.set_ylim([0, 1.7])

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
    ax2.set_ylim([0, 1.8])

axs[0].set_title("Scalability Analyses", fontsize=16)
axs[-1].set_xlabel('Time (seconds)', fontsize=14)


plt.savefig("robustness.pdf", dpi=200)
plt.show()
