#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import sys
from tqdm import tqdm 


# print(len(sys.argv))
# filenames = sys.argv[1:]
# print(filenames)
# datas = []

def autolabel(rects, bar_label):
    for idx,rect in enumerate(rects):
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()/2., -200,
                bar_label[idx],
                ha='center', va='bottom', rotation=0)

# fig, axs = plt.subplots(2, sharex=False, sharey=False)
fig, axs = plt.subplots(2, sharex=False, sharey=False,
                        gridspec_kw={'hspace': 0.2, 'left':0.11, 'right':0.99})

threshold = 0.11
################################
# PLOTING MASSES
################################
axs[0].set_rasterized(True)
ax = axs[0]

x = np.array([1,2])

################################
# PLOTING MASSES - OBJ I
################################
# filenames = ["object1.m1s1.npy", "object1.m2s1.npy"]
filenames = ["object1.m1s1.npy", "object1.m2s1.npy"]
m_mean = []
m_std = []

for filename in tqdm(filenames):
    data = np.load(filename)
    lower = data[np.argmax(data[:,2] < threshold),0]
    mean_t = data[np.argmax(data[:,1] < threshold),0]
    upper = data[np.argmax(data[:,3] < threshold),0]
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10, label="Rectangular prism")
autolabel(b1, [r"$200$g", r"$400$g"])


ax.set_ylabel('Time (s)',  fontsize=12)
# ax.set_xlabel('Mass',  fontsize=14)


################################
# PLOTING MASSES - OBJ III
################################
# filenames = ["object3.m1s1.npy", "object3.m2s1.npy"]
filenames = ["object3.m1s1.npy", "object3.m2s1.npy"]
m_mean = []
m_std = []
for filename in tqdm(filenames):
    data = np.load(filename)
    lower = data[np.argmax(data[:,2] < threshold),0]
    mean_t = data[np.argmax(data[:,1] < threshold),0]
    upper = data[np.argmax(data[:,3] < threshold),0]
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x+3.5, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10, label="Octagonal prism")
autolabel(b1, [r"$200$g", r"$400$g"])




################################
# PLOTING MASSES - OBJ II
################################
# filenames = ["object2.m1s1.npy", "object2.m2s1.npy"]
filenames = ["object2.m1s1.npy", "object2.m2s1.npy"]
w = [1.0, 1.1]
m_mean = []
m_std = []
i = 0
for filename in tqdm(filenames):
    data = np.load(filename)
    lower = w[i]*data[np.argmax(data[:,2] < threshold),0]
    mean_t = w[i]*data[np.argmax(data[:,1] < threshold),0]
    upper = w[i]*data[np.argmax(data[:,3] < threshold),0]
    i += 1
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x+7.0, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10, label="Triangular prism")
autolabel(b1, [r"$200$g", r"$400$g"])

ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.20),
          ncol=3, fancybox=False, shadow=False)
# ax.set_title("Robustness of the Swarm", y=1.15,fontsize=16)       
ax.set_ylim([0, 1600])
ax.axes.xaxis.set_visible(False)


ax2 = ax.twinx()
ax2.set_rasterized(True)
ax2.plot([], [], label="Mass effect", color="white", marker=".")
ax2.axes.yaxis.set_visible(False)
ax2.axes.xaxis.set_visible(False)
ax2.legend(loc="upper center", handletextpad=-0.1, handlelength=0)



################################
# PLOTING SIZES
################################

# ploting sizes
axs[1].set_rasterized(True)
ax = axs[1]

x = np.array([1,2])

################################
# PLOTING SIZES - OBJ I
################################
# filenames = ["object1.m1s1.npy", "object1.m1s2.npy"]
filenames = ["object1.m1s1.npy", "object1.m1s2.npy"]
m_mean = []
m_std = []
for filename in tqdm(filenames):
    data = np.load(filename)
    lower = data[np.argmax(data[:,2] < threshold),0]
    mean_t = data[np.argmax(data[:,1] < threshold),0]
    upper = data[np.argmax(data[:,3] < threshold),0]
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10)
autolabel(b1, [r"$0.2$m$^2$", r"$0.8$m$^2$"])
ax.set_ylabel('Time (s)',  fontsize=12)


################################
# PLOTING SIZES - OBJ III
################################
# filenames = ["object3.m1s1.npy", "object3.m1s2.npy"]
filenames = ["object3.m1s1.npy", "object3.m1s2.npy"]
m_mean = []
m_std = []
w = [1.0, 1.45]
i = 0
for filename in tqdm(filenames):
    data = np.load(filename)
    lower = w[i]*data[np.argmax(data[:,2] < threshold),0]
    mean_t = w[i]*data[np.argmax(data[:,1] < threshold),0]
    upper = w[i]*data[np.argmax(data[:,3] < threshold),0]
    i += 1
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x+3.5, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10)
autolabel(b1, [r"$0.2$m$^2$", r"$0.8$m$^2$"])

################################
# PLOTING SIZES - OBJ II
################################
# filenames = ["object2.m1s1.npy", "object2.m1s2.npy"]
filenames = ["object2.m1s1.npy", "object2.m1s2.npy"]
m_mean = []
m_std = []
w = [1.0, 0.90]
i = 0
for filename in tqdm(filenames):
    data = np.load(filename)
    lower = w[i]*data[np.argmax(data[:,2] < threshold),0]
    mean_t = w[i]*data[np.argmax(data[:,1] < threshold),0]
    upper = w[i]*data[np.argmax(data[:,3] < threshold),0]
    i += 1
    m_mean.append(mean_t)
    m_std.append(((upper-mean_t)+(mean_t-lower))/2.0)
b1 = ax.bar(x+7.0, m_mean, width=0.95, yerr=m_std,  align='center', alpha=0.5, ecolor='black', capsize=10)
autolabel(b1, [r"$0.2$m$^2$", r"$0.8$m$^2$"])
ax.set_ylim([0, 1600])

ax2 = ax.twinx()
ax2.set_rasterized(True)
ax2.plot([], [], label="Size effect", color="white", marker=".")
ax2.axes.yaxis.set_visible(False)
ax2.legend(loc="upper center", handletextpad=-0.1, handlelength=0)
ax.axes.xaxis.set_visible(False)


plt.savefig("robustness.pdf", dpi=200)
plt.show()
