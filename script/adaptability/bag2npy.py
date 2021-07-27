#!/usr/bin/env python 

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import sys

import rosbag


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h


if (len(sys.argv) != 2):
    print("Wrong syntaxe: python eval_1_plot.py <file>.(bag|npy)")
    exit() 
# Choose process file
data = []
figfile = 'out.png'
# 1) Get data from rosbag and save to npy format (faster)
if "bag" in sys.argv[1]:
    print("Loading data: " + sys.argv[1])
    bag = rosbag.Bag(sys.argv[1])
    figfile = sys.argv[1].replace("bag", "png")
    topic_name = "/object_state"
    t_start = 0
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        t_ = float(str(t))
        if t_start == 0:
            t_start = t_
        if msg.x < 3:
            data.append([(t_-t_start)/1e9, msg.x, msg.y])
    data = np.array(data)
    np.save(sys.argv[1].replace("bag", "npy"), data)

# 2) Get data from npy format (faster)
elif "npy" in sys.argv[1]:
    print("Loading data: "+ sys.argv[1])
    figfile = sys.argv[1].replace("npy", "png")
    data = np.load(sys.argv[1])
    print(data)


# Ploting data
fig = plt.figure(dpi=200)
ax = fig.add_subplot(111)
cut = 2000000
color = 'tab:red'
ax.plot(data[:cut,0], data[:cut,2], label='object velocity', color=color)
ax.set_xlabel('time (seconds)')
ax.set_ylabel('object velocity (cm/s)', color=color)
ax.tick_params(axis='y', labelcolor=color)
ax.set_ylim([0, 3.2])


ax2 = ax.twinx()
color = 'tab:blue'
ax2.set_ylabel('distance to goal (m)', color=color)
ax2.plot(data[:cut,0], data[:cut,1], label='distance to goal', color=color)
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim([0, 2.2])

ax.set_title("Convergence Analyses")

# ax.set_xscale('symlog')
# ax.set_yscale('symlog')
# ax.legend()
# ax2.legend()
plt.savefig(figfile, dpi=200)
# plt.show()








#############
# #!/usr/bin/env python 

# import matplotlib.pyplot as plt
# import numpy as np
# import scipy.stats
# import sys

# import rosbag


# def mean_confidence_interval(data, confidence=0.95):
#     a = 1.0 * np.array(data)
#     n = len(a)
#     m, se = np.mean(a), scipy.stats.sem(a)
#     h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
#     return m, m-h, m+h


# if (len(sys.argv) != 2):
#     print("Wrong syntaxe: python eval_1_plot.py <file>.(bag|npy)")
#     exit() 
# # Choose process file
# data = []
# figfile = 'out.png'
# # 1) Get data from rosbag and save to npy format (faster)
# if "bag" in sys.argv[1]:
#     print("Loading data: "+ sys.argv[1])
#     bag = rosbag.Bag(sys.argv[1])
#     figfile = sys.argv[1].replace("bag", "png")
#     topic_name = "/gazebo/model_states"
#     t_start = 0
#     net_vel = 0
#     gain = 0.05
#     for topic, msg, t in bag.read_messages(topics=[topic_name]):
#         t_ = float(str(t))
#         if t_start == 0:
#             t_start = t_
#         # search goal location
#         goal = msg.pose[msg.name.index("cardboard_box_target")].position
#         # search object idx
#         # print(msg.name)
#         idx = msg.name.index("cardboard_box")
#         pos = msg.pose[idx].position
#         dx = goal.x - pos.x
#         dy = goal.y - pos.y
#         dist = np.sqrt(dx**2 + dy**2)
#         # print(dist)
#         vel = msg.twist[idx]
#         net_vel_new = np.sqrt(vel.linear.x**2 + vel.linear.y**2)
#         net_vel = net_vel * (1-gain) + net_vel_new * (gain)
#         # print(net_vel)
#         data.append([(t_-t_start)/1e9, dist, net_vel])
#     data = np.array(data)
#     np.save(sys.argv[1].replace("bag", "npy"), data)

# # 2) Get data from npy format (faster)
# elif "npy" in sys.argv[1]:
#     print("Loading data: "+ sys.argv[1])
#     figfile = sys.argv[1].replace("npy", "png")
#     data = np.load(sys.argv[1])
#     print(data)


# # Ploting data
# fig = plt.figure(dpi=200)
# ax = fig.add_subplot(111)
# cut = 2000000
# color = 'tab:red'
# ax.plot(data[:cut,0], data[:cut,2]*100.0, label='object velocity', color=color)
# ax.set_xlabel('time (seconds)')
# ax.set_ylabel('object velocity (cm/s)', color=color)
# ax.tick_params(axis='y', labelcolor=color)
# ax.set_ylim([0, 3.2])


# ax2 = ax.twinx()
# color = 'tab:blue'
# ax2.set_ylabel('distance to goal (m)', color=color)
# ax2.plot(data[:cut,0], data[:cut,1], label='distance to goal', color=color)
# ax2.tick_params(axis='y', labelcolor=color)
# ax2.set_ylim([0, 2.2])

# ax.set_title("Convergence Analyses")

# # ax.set_xscale('symlog')
# # ax.set_yscale('symlog')
# # ax.legend()
# # ax2.legend()
# plt.savefig(figfile, dpi=200)
# # plt.show()