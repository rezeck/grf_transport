#!/usr/bin/env python 

import numpy as np
import math
import os
import sys
import math
import argparse
import sys
import rospy

parser = argparse.ArgumentParser(description='Create a launch file that spawn multiples robots.')
parser.add_argument('--robots', help='Number of robots.', default=4, type=int)
parser.add_argument('--groups', help='Number of groups.', default=1, type=int)
parser.add_argument('--sensing', help='Sensing area in meters.', default=0.50, type=float)
parser.add_argument('--worldsize', help='Size of the world meters x meters.', default=4.0, type=float)
parser.add_argument('--safezone', help='Safe zone to avoid collisions.', default=0.15, type=float)
parser.add_argument('--dt', help='Update rate for kinematic models predictions.', default=0.01, type=float)
parser.add_argument('--mass', help='Mass parameters used to CB-potential.', default=5.0, type=float)
parser.add_argument('--vmax', help='Maximal velocity performed by the robot.', default=0.3, type=float)
parser.add_argument('--seed', help='Random SEED.', default=0, type=int)
args = parser.parse_args()

# Instanciate the random position of the robots 
np.random.seed(args.seed)

def get_sample ():
    sample = 0.9 * args.worldsize * (np.random.uniform(size=2) - 0.5) # position
    # print(sample)
    return sample

q = np.array([get_sample()])
counter = 1
while counter < int(args.robots):
    s = get_sample()
    d = q - s
    dist = np.sqrt(np.power(d[:,0],2) + np.power(d[:,1],2))
    if np.any(dist > 0.30):
        q = np.append(q, [s], axis=0)
        counter  += 1
# print(q)

launch_file = """<launch>
    <param name="robots" type="int" value="$ROBOTS"/>
    <param name="groups" type="int" value="$GROUPS"/>
    <param name="sensing" type="double" value="$SENSING"/>
    <param name="worldsize" type="double" value="$WORLDSIZE"/>
    <param name="safezone" type="double" value="$SAFEZONE"/>
    <param name="dt" type="double" value="$DT"/>
    <param name="mass" type="double" value="$MASS"/>
    <param name="vmax" type="double" value="$VMAX"/>
$INCLUDES
</launch>"""
launch_file = launch_file.replace("$ROBOTS", str(args.robots))
launch_file = launch_file.replace("$GROUPS", str(args.groups))
launch_file = launch_file.replace("$SENSING", str(args.sensing))
launch_file = launch_file.replace("$WORLDSIZE", str(args.worldsize))
launch_file = launch_file.replace("$SAFEZONE", str(args.safezone))
launch_file = launch_file.replace("$DT", str(args.dt))
launch_file = launch_file.replace("$MASS", str(args.mass))
launch_file = launch_file.replace("$VMAX", str(args.vmax))

include_patch = """
    <include file="$(find hero_gazebo)/launch/hero_spawn.launch">
        <arg name="x" value="$X"/>
        <arg name="y" value="$Y"/>
        <arg name="z" value="$Z"/>
        <arg name="id" value="$ID"/>
        <arg name="type" value="$TYPE"/>
    </include>
"""
include_patches = """"""

r = args.robots/args.groups
for i in range(0, int(args.robots)):
    temp = include_patch
    temp = temp.replace("$X", str(q[i,0]))
    temp = temp.replace("$Y", str(q[i,1]))
    temp = temp.replace("$Z", str(0.55))
    temp = temp.replace("$ID", str(i))
    temp = temp.replace("$TYPE", str(int(math.floor(i/r))))
    include_patches += temp

launch_file = launch_file.replace("$INCLUDES", include_patches)
print(launch_file)