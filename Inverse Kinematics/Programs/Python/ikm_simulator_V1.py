# -*- coding: utf-8 -*-
"""
Created on Mon Feb 20 06:58:47 2023

Simulation environment for inverse kinematic model of a single leg from a hexapod.
This script creates an animated plot with adjustable variables to verify the 
behavior of the inverse kinematic model.

@author: Philipp Schulz
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
import time

# mode of simulation
mode = 1 # 0 for angle slider, 1 for xyz slider

# constants
leg_count = 6
parameter_count = 3

# variables for axes
angles = np.zeros(3)
# empty singular transforms
leg_transforms = [[np.zeros((4,4)) for j in range(0,5,1)] for i in range(0,leg_count,1)] # j: each system, i: each leg
# empty product transforms
sum_transforms =[[np.zeros((4,4)) for j in range(0,4,1)] for i in range(0,leg_count,1)] # j: each system, i: each leg
# angles of all leg joints
all_angles = np.zeros((leg_count,parameter_count))
# XYZ targets relative to coordinate system of each leg
kartesian = np.zeros(parameter_count*2)
# kartesian helper arrays
defaultLegTransform = [44,0,-69.5,0,0,0]
kartesianX = np.zeros(leg_count)
kartesianY = np.zeros(leg_count)
kartesianZ = np.zeros(leg_count)
kartesianAngles = np.zeros(leg_count*parameter_count)
kartesian_ref = np.zeros((leg_count,3))

# helper variables for plotting
ref_lines = [[] for i in range(0,7,1)]
quivers = [0 for i in range(0,25,1)]
lines = [[] for i in range(0,24,1)]
quiv_colors = ['r','g','b']
# ranges for reference lines
x_lim = [-200,200]
y_lim = [-200,200]
z_lim = [-200,200]

# requirements to allow compliance of code between Python and C++
sqrt = np.sqrt
PI = np.pi
sin = np.sin
cos = np.cos
tan = np.tan
asin = np.arcsin
acos = np.arccos
atan = np.arctan

# variables for angle cycling
dir1 = 1
dir2 = 1
dir3 = 1

# variables to control walking behavior
is_walking = False
walking_phases = np.zeros(leg_count)
delay_legs = True
last_timestamp = 0
walking_mode = 1

# function for cycling through all angles to test plot
def cycle_angles():
    global all_angles
    global dir1
    global dir2
    global dir3
    s11 = np.radians(-90)
    s12 = np.radians(90)
    s21 = np.radians(-90)
    s22 = np.radians(90)
    s31 = np.radians(-45)
    s32 = np.radians(135)
    step = np.radians(2)
    if dir1:
        all_angles[:,0] += step
        if all_angles[0,0] >= s12:
            dir1 = 0
    else:
        all_angles[:,0] -= step
        if all_angles[0,0] <= s11:
            dir1 = 1
    if dir2:
        all_angles[:,1] += step
        if all_angles[0,1] >= s22:
            dir2 = 0
    else:
        all_angles[:,1] -= step
        if all_angles[0,1] <= s21:
            dir2 = 1
    if dir3:
        all_angles[:,2] += step
        if all_angles[0,2] >= s32:
            dir3 = 0
    else:
        all_angles[:,2] -= step
        if all_angles[0,2] <= s31:
            dir3 = 1

# function for continuously rotating the complete hexapod
def cycle_tilts():
    global kartesian
    step = np.radians(2)
    kartesian[3] += step
    kartesian[4] += step
    kartesian[5] += step
    

# function to generate a transform from given D&H parameters
def dh_transform(theta, s, a, alpha):
    # 1. rotate around z by theta until x are parallel
    # 2. move along z by s until x intersect
    # 3. move along x by a until x are identical
    # 4. rotate around x by alpha until z are identical
    ca = np.cos(alpha);
    sa = np.sin(alpha);
    ct = np.cos(theta);
    st = np.sin(theta);
    return np.asarray([[ct, -st*ca, st*sa, a*ct],[st, ct*ca, -ct*sa, a*st],[0, sa, ca, s],[0, 0, 0, 1]])

# function to draw initial setup to speed up animation
def initial_draw():
    global leg_transforms
    global sum_transforms
    global ref_lines
    global quivers
    global lines
    if mode:
        # calculate inverse kinematics
        # inverse_kinematics_1()
        # inverse_kinematics_2()
        # inverse_kinematics_3()
        inverse_kinematics_4()
    else:
        # animation testing
        cycle_angles()
    # recalculate transforms
    update_transforms()
    if mode:
        z = defaultLegTransform[2] + 12.6
        # draw reference lines of target coordinates
        # x lines
        ref_lines[0] = ax.plot([(37.5+28.5+32.25+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z],color="black",linestyle="--")
        ref_lines[1] = ax.plot([-(37.5+28.5+32.25+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z],color="black",linestyle="--")
        ref_lines[2] = ax.plot([(37.5+32.25+20.15+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z],color="black",linestyle="--")
        ref_lines[3] = ax.plot ([-(37.5+32.25+20.15+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z],color="black",linestyle="--")
        # y lines
        ref_lines[4] = ax.plot(y_lim,
                                 [(56+4.24+20.15+defaultLegTransform[1]) for i in range(0,2,1)],
                                 [z,z],color="black",linestyle="--")
        ref_lines[5] = ax.plot(y_lim,
                                 [-(56+4.24+20.15+defaultLegTransform[1]) for i in range(0,2,1)],
                                 [z,z],color="black",linestyle="--")
        ref_lines[6] = ax.plot(y_lim,[kartesian[1]+defaultLegTransform[1] for i in range(0,2,1)],
                                 [z,z],color="black",linestyle="--")
    # draw coordinate system origin to robot, only required once
    quiv_set = []
    for i in range(0,3,1):
        quiv_set.append(ax.quiver(leg_transforms[0][0][0,-1],leg_transforms[0][0][1,-1],leg_transforms[0][0][2,-1],
                               leg_transforms[0][0][i,0:3][0],leg_transforms[0][0][i,0:3][1],leg_transforms[0][0][i,0:3][2],
                               color=quiv_colors[i],length=25,normalize=True))
    quivers[0] = quiv_set
    # loop over all legs
    colors = ["black","blue","black","blue"]
    for i in range(0,len(sum_transforms),1):
        # plot first line
        lines[i*len(sum_transforms[i])] = ax.plot([0,sum_transforms[i][0][0,-1]],
                                                  [0,sum_transforms[i][0][1,-1]],
                                                  [0,sum_transforms[i][0][2,-1]],color=colors[0])
        quiv_set = []
        for k in range(0,3,1):
            quiv_set.append(ax.quiver(sum_transforms[i][0][0,-1],
                        sum_transforms[i][0][1,-1],sum_transforms[i][0][2,-1],
                        sum_transforms[i][0][0:3,k][0],sum_transforms[i][0][0:3,k][1],sum_transforms[i][0][0:3,k][2],
                        color=quiv_colors[k],length=25,normalize=True))
        quivers[i*(len(sum_transforms[i]))+1] = quiv_set
        # loop over rest of sum_transforms
        for j in range(1,len(sum_transforms[i]),1):
            # plot line
            lines[i*len(sum_transforms[i])+j] = ax.plot([sum_transforms[i][j-1][0,-1],
                        sum_transforms[i][j][0,-1]],[sum_transforms[i][j-1][1,-1],sum_transforms[i][j][1,-1]],
                        [sum_transforms[i][j-1][2,-1],sum_transforms[i][j][2,-1]],color=colors[j])
            # draw coordinate systems
            quiv_set = []
            for k in range(0,3,1):
                quiv_set.append(ax.quiver(sum_transforms[i][j][0,-1],
                            sum_transforms[i][j][1,-1],sum_transforms[i][j][2,-1],
                            sum_transforms[i][j][0:3,k][0],sum_transforms[i][j][0:3,k][1],sum_transforms[i][j][0:3,k][2],
                            color=quiv_colors[k],length=25,normalize=True))
            quivers[i*(len(sum_transforms[i]))+j+1] = quiv_set
    # set axis limits to fix 3d space
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)
    ax.set_zlim(z_lim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    # adjust lines list
    for i in range(0,len(lines),1):
        lines[i] = lines[i][0]
    for i in range(0,len(ref_lines),1):
        if not ref_lines[i] == []:
            ref_lines[i] = ref_lines[i][0]
        else:
            break;
    

# function to update the plot
def animate(val):
    global kartesian
    global leg_transforms
    global sum_transforms
    global ref_lines
    global quivers
    global lines
    # start_time = time.time()
    if mode:
        # do it for the meme
        # cycle_tilts()
        if walking_mode == 0:
            # animate walking
            walking()
            a=1
        elif walking_mode == 1:
            # animate walking with second iteration
            predefine_positions()
            walking()
        
        # calculate inverse kinematics
        # inverse_kinematics_1()
        # inverse_kinematics_2()
        # inverse_kinematics_3()
        inverse_kinematics_4()
    #else:
        # animation testing
        #cycle_angles()
    # recalculate transforms
    update_transforms()
    # draw reference lines
    if mode:
        z = defaultLegTransform[2]+12.6
        # draw reference lines of target coordinates
        # x lines
        ref_lines[0].set_data_3d([(37.5+32.25+28.5+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z])
        ref_lines[1].set_data_3d([-(37.5+32.25+28.5+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z])
        ref_lines[2].set_data_3d([(37.5+32.25+20.15+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z])
        ref_lines[3].set_data_3d([-(37.5+32.25+20.15+defaultLegTransform[0]) for i in range(0,2,1)],
                                 x_lim,[z,z])
        # y lines
        ref_lines[4].set_data_3d(y_lim,
                                 [(56+4.24+20.15+defaultLegTransform[1]) for i in range(0,2,1)],
                                 [z,z])
        ref_lines[5].set_data_3d(y_lim,
                                 [-(56+4.24+20.15+defaultLegTransform[1]) for i in range(0,2,1)],
                                 [z,z])
        ref_lines[6].set_data_3d(y_lim,[defaultLegTransform[1] for i in range(0,2,1)],
                                 [z,z])
    # remove all quivers
    for i in range(0,len(quivers),1):
        for j in range(0,len(quivers[i]),1):
            quivers[i][j].remove()
    # draw coordinate system origin to robot, only required once
    quiv_set = []
    for i in range(0,3,1):
        quiv_set.append(ax.quiver(leg_transforms[0][0][0,-1],leg_transforms[0][0][1,-1],leg_transforms[0][0][2,-1],
                               leg_transforms[0][0][i,0:3][0],leg_transforms[0][0][i,0:3][1],leg_transforms[0][0][i,0:3][2],
                               color=quiv_colors[i],length=25,normalize=True))
    quivers[0] = quiv_set
    # loop over all legs
    for i in range(0,len(sum_transforms),1):
        # plot first line
        #"""
        lines[i*len(sum_transforms[i])].set_data_3d([kartesian[0],sum_transforms[i][0][0,-1]],
                                                    [kartesian[1],sum_transforms[i][0][1,-1]],
                                                    [kartesian[2],sum_transforms[i][0][2,-1]])#"""
        quiv_set = []
        for k in range(0,3,1):
            quiv_set.append(ax.quiver(sum_transforms[i][0][0,-1],
                        sum_transforms[i][0][1,-1],sum_transforms[i][0][2,-1],
                        sum_transforms[i][0][0:3,k][0],sum_transforms[i][0][0:3,k][1],sum_transforms[i][0][0:3,k][2],
                        color=quiv_colors[k],length=25,normalize=True))
        quivers[i*(len(sum_transforms[i]))+1] = quiv_set
        #ax.plot([0,sum_transforms[i][0][0,-1]],[0,sum_transforms[i][0][1,-1]],[0,sum_transforms[i][0][2,-1]],color=colors[0])
        # loop over rest of sum_transforms
        for j in range(1,len(sum_transforms[i]),1):
            # plot line
            lines[i*len(sum_transforms[i])+j].set_data_3d([sum_transforms[i][j-1][0,-1],
                        sum_transforms[i][j][0,-1]],[sum_transforms[i][j-1][1,-1],sum_transforms[i][j][1,-1]],
                        [sum_transforms[i][j-1][2,-1],sum_transforms[i][j][2,-1]])
            # draw coordinate system
            quiv_set = []
            for k in range(0,3,1):
                quiv_set.append(ax.quiver(sum_transforms[i][j][0,-1],
                            sum_transforms[i][j][1,-1],sum_transforms[i][j][2,-1],
                            sum_transforms[i][j][0:3,k][0],sum_transforms[i][j][0:3,k][1],sum_transforms[i][j][0:3,k][2],
                            color=quiv_colors[k],length=25,normalize=True))
            quivers[i*(len(sum_transforms[i]))+j+1] = quiv_set
    # stop_time = time.time()
    #print(str(round((stop_time-start_time)*1000,2))+" ms")

# function to update the slider values
def update_slider_angle(val):
    global all_angles
    value1 = np.radians(slider1.val)
    value2 = np.radians(slider2.val)
    value3 = np.radians(slider3.val)
    for i in range(0,leg_count,1):
        all_angles[i][0] = value1
        all_angles[i][1] = value2
        all_angles[i][2] = value3

# function to update the slider values
def update_slider_xyz(val):
    global kartesian
    kartesian[0] = slider1.val
    kartesian[1] = slider2.val
    kartesian[2] = slider3.val
    kartesian[3] = np.radians(slider4.val)
    kartesian[4] = np.radians(slider5.val)
    kartesian[5] = np.radians(slider6.val)

# function to update transforms for leg 4
def update_transforms():
    global all_angles
    global kartesian
    global leg_transforms
    global sum_transforms
    # transform constants based on geometry
    x_r1 = 35.74+20.15
    x_r2 = 37.5+28.5
    y_r1 = 56+4.24+20.15
    y_r1a = y_r1 - x_r1
    x_r3 = np.sqrt(x_r1**2+x_r1**2)
    z_r = 12.6
    l2 = 32.25
    l3 = 44
    l4 = 69.5
    a = kartesian[3]
    b = kartesian[4]
    g = kartesian[5]
    # loop over all legs
    for i in range(0,len(leg_transforms),1):
        # origin to robot
        # leg_transforms[i][0] = dh_transform(0,0,0,0) # without rotational angles
        # """
        leg_transforms[i][0] = np.asarray([[cos(g)*cos(b),-sin(g)*cos(a)+cos(g)*sin(b)*sin(a),sin(g)*sin(a)+cos(g)*sin(b)*cos(a),kartesian[0]],
                                           [sin(g)*cos(b),cos(g)*cos(a)+sin(g)*sin(b)*sin(a), -cos(g)*sin(a)+sin(g)*sin(b)*cos(a),kartesian[1]],
                                           [-sin(b),cos(b)*sin(a),cos(b)*cos(a),kartesian[2]],
                                           [0,0,0,1]]) # with rotational angles #"""
        # robot to joint 1
        if i == 0:
            # 1. rotate around z by theta until x are parallel
            # 2. move along z by s until x intersect
            # 3. move along x by a until x are identical
            # 4. rotate around x by alpha until z are identical
            t1 = dh_transform(np.radians(90),z_r,y_r1a,0)
            t2 = dh_transform(np.radians(45),0,x_r3,0)
            leg_transforms[i][1] = np.dot(t1,t2)
        elif i == 1:
            t1 = dh_transform(np.radians(90),z_r,y_r1a,0)
            t2 = dh_transform(-np.radians(45),0,x_r3,0)
            leg_transforms[i][1] = np.dot(t1,t2)
            #leg_transforms[i][1] = dh_transform(np.arctan(y_r1/x_r1),z_r,x_r3,0)
        elif i == 2:
            leg_transforms[i][1] = dh_transform(np.radians(180),z_r,x_r2,0)
        elif i == 3:
            leg_transforms[i][1] = dh_transform(0,z_r,x_r2,0)
        elif i == 4:
            t1 = dh_transform(-np.radians(90),z_r,y_r1a,0)
            t2 = dh_transform(-np.radians(45),0,x_r3,0)
            leg_transforms[i][1] = np.dot(t1,t2)
            #leg_transforms[i][1] = dh_transform(-np.radians(90)-(np.radians(90)-np.arctan(y_r1/x_r1)),z_r,x_r3,0)
        elif i == 5:
            t1 = dh_transform(-np.radians(90),z_r,y_r1a,0)
            t2 = dh_transform(np.radians(45),0,x_r3,0)
            leg_transforms[i][1] = np.dot(t1,t2)
            #leg_transforms[i][1] = dh_transform(-np.arctan(y_r1/x_r1),z_r,x_r3,0)
        # split by side of robot
        if i%2 == 0:
            # joint 1 to joint 2
            leg_transforms[i][2] = dh_transform(-all_angles[i][0],0,l2,np.pi/2)
            # joint 2 to joint 3
            leg_transforms[i][3] = dh_transform(all_angles[i][1],0,l3,0)
            # joint 3 to end
            leg_transforms[i][4] = dh_transform(-all_angles[i][2],0,l4,-np.pi/2)
        else:
            # joint 1 to joint 2
            leg_transforms[i][2] = dh_transform(all_angles[i][0],0,l2,-np.pi/2)
            # joint 2 to joint 3
            leg_transforms[i][3] = dh_transform(-all_angles[i][1],0,l3,0)
            # joint 3 to end
            leg_transforms[i][4] = dh_transform(all_angles[i][2],0,l4,np.pi/2)
        # determine sum_transforms
        sum_transforms[i][0] = np.dot(leg_transforms[i][0],leg_transforms[i][1])
        # loop over rest of sum_transforms
        for j in range(1,len(sum_transforms[i]),1):
            sum_transforms[i][j] = np.dot(sum_transforms[i][j-1],leg_transforms[i][j+1])

# function to generate the reference leg positions
def predefine_positions():
    global kartesian_ref
    # constants used in the calculations
    l1m = 32.25
    l1corr = 0.09
    # get requested coordinates and angles first
    x = defaultLegTransform[0];
    y = defaultLegTransform[1];
    z = defaultLegTransform[2];
    a = defaultLegTransform[3];
    b = defaultLegTransform[4];
    g = defaultLegTransform[5];
    # ONLY FOR PYTHON
    a = np.degrees(a)
    b = np.degrees(b)
    g = np.degrees(g)
    # set supposed kartesian coordinates for all legs
    # -------------leg 1-------------
    atemp = np.arctan2(y,x+l1m);
    kartesian_ref[0,0] = -(sqrt(pow(x+l1m,2)+pow(y,2)))*cos(atemp);
    kartesian_ref[0,1] = (sqrt(pow(x+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 2-------------
    atemp = np.arctan2(y,x+l1m);
    kartesian_ref[1,0] = (sqrt(pow(x+l1m,2)+pow(y,2)))*cos(atemp);
    kartesian_ref[1,1] = (sqrt(pow(x+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 3-------------
    kartesian_ref[2,0] = -(x+l1m);
    kartesian_ref[2,1] = y;
    # -------------leg 4-------------
    kartesian_ref[3,0] = x+l1m;
    kartesian_ref[3,1] = y;
    # -------------leg 5-------------
    atemp = np.arctan2(y,x+l1m);
    kartesian_ref[4,0] = -(sqrt(pow(x+l1m,2)+pow(y,2)))*cos(atemp);
    kartesian_ref[4,1] = (sqrt(pow(x+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 6-------------
    atemp = np.arctan2(y,x+l1m);
    kartesian_ref[5,0] = (sqrt(pow(x+l1m,2)+pow(y,2)))*cos(atemp);
    kartesian_ref[5,1] = (sqrt(pow(x+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------z-------------
    kartesian_ref[0,2] = z;
    kartesian_ref[1,2] = z;
    kartesian_ref[2,2] = z;
    kartesian_ref[3,2] = z;
    kartesian_ref[4,2] = z;
    kartesian_ref[5,2] = z;

# function to implement walking procedure for all legs
def walking():
    global kartesianX
    global kartesianY
    global kartesianZ
    global is_walking
    global walking_phases
    global last_timestamp
    global delay_legs
    # distances the leg should travel
    y_delta = 30
    z_delta = 30
    stepsize = 5
    step_time = 2 # cycle time for one step
    # indices of leg groups
    indices1 = [0,3,4]
    indices2 = [1,2,5]
    # check flag for walking
    if not is_walking:
        if walking_mode == 1:
            # save timestamp for starting
            last_timestamp = time.time()
            # program start, time to set flags
            for i in range(0,len(indices1),1):
                walking_phases[indices1[i]] = step_time/2
                walking_phases[indices2[i]] = step_time/2#"""
        # set flag for walking
        is_walking = True
    if walking_mode == 1:
        # determine time since last loop
        current_timestamp = time.time()
        time_delay = current_timestamp-last_timestamp
        # assign last timestamp
        last_timestamp = current_timestamp
        # add time_delay to walking phases
        walking_phases += time_delay
        # loop over indices
        for i in range(0,len(indices1),1):
            # check if phase is longer than step_time
            if walking_phases[indices1[i]] > step_time:
                walking_phases[indices1[i]] -= step_time
            # check for first leg set if first or second phase is reached
            if walking_phases[indices1[i]] < step_time/2:
                # use first equations for current positions
                kartesianY[indices1[i]] = kartesian_ref[indices1[i],1] + y_delta*(4*walking_phases[indices1[i]]/step_time-1)
                kartesianZ[indices1[i]] = kartesian_ref[indices1[i],2] + z_delta*sin(2*PI/step_time*walking_phases[indices1[i]])
            elif walking_phases[indices1[i]] <= step_time:
                kartesianY[indices1[i]] = kartesian_ref[indices1[i],1] + y_delta*(3-4*walking_phases[indices1[i]]/step_time)
                # kartesianZ[indices1[i]] += 0
            # check for second leg set if first or second phase is reached
            if walking_phases[indices2[i]] > step_time:
                walking_phases[indices2[i]] -= step_time
            if walking_phases[indices2[i]] < step_time/2:
                # use first equations for current positions
                kartesianY[indices2[i]] = kartesian_ref[indices1[i],1] + y_delta*(1-4*walking_phases[indices2[i]]/step_time)
                # kartesianZ[indices1[i]] += 0
            elif walking_phases[indices2[i]] <= step_time:
                kartesianY[indices2[i]] = kartesian_ref[indices1[i],1] + y_delta*(4*walking_phases[indices2[i]]/step_time-3)
                kartesianZ[indices2[i]] = kartesian_ref[indices1[i],2] + z_delta*sin(2*PI/step_time*(walking_phases[indices2[i]]-step_time/2))
    elif walking_mode == 0:
        # cycle through all legs and set their phases
        for i in range(0,leg_count,1):
            # check if legs need to be delayed
            if not (delay_legs and (i==1 or i==2 or i == 5)):
                # phase 1 - lift leg up
                if walking_phases[i] == 0:
                    kartesianZ[i] += stepsize
                    if kartesianZ[i] >= kartesian_ref[i,2] + z_delta:
                        walking_phases[i] = 1
                # phase 2 - move leg forward
                elif walking_phases[i] == 1:
                    kartesianY[i] += stepsize
                    if kartesianY[i] >= kartesian_ref[i,1] + y_delta:
                        walking_phases[i] = 2
                # phase 3 - move leg down
                elif walking_phases[i] == 2:
                    kartesianZ[i] -= stepsize
                    if kartesianZ[i] <= kartesian_ref[i,2]:
                        walking_phases[i] = 3
                # phase 4 - move leg backward
                elif walking_phases[i] == 3:
                    # reset delay flag for walking start
                    if delay_legs:
                        delay_legs = False
                    kartesianY[i] -= stepsize
                    if kartesianY[i] <= kartesian_ref[i,1] - y_delta:
                        walking_phases[i] = 0

# signum function
def sgn(x):
  if(x == 0):
    return 1;
  else:
    return x/abs(x)

# function to implement the inverse kinematics that were originally put onto the hexapod
def inverse_kinematics_1():
    global all_angles
    global kartesianX
    global kartesianY
    global kartesianZ
    global kartesian
    x = kartesian[0]
    y = kartesian[1]
    z = kartesian[2]
    # get requested coordinates and angles first
    x = defaultLegTransform[0]+kartesian[0];
    y = defaultLegTransform[1]+kartesian[1];
    z = defaultLegTransform[2]+kartesian[2];
    #a = defaultLegTransform[3]+kartesian[3];
    #b = defaultLegTransform[4]+kartesian[4];
    #g = defaultLegTransform[5]+kartesian[5];
    # set supposed kartesian coordinates for all legs
    # -------------x-------------
    kartesianX[0] = -sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(135.0*(PI/180.0));
    kartesianX[1] = sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(45.0*(PI/180.0));
    kartesianX[2] = -x;
    kartesianX[3] = x;
    kartesianX[4] = sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(-135.0*(PI/180.0));
    kartesianX[5] = -sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(-45.0*(PI/180.0));
    # -------------y-------------
    kartesianY[0] = -sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(135.0*(PI/180.0));
    kartesianY[1] = sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(45.0*(PI/180.0));
    kartesianY[2] = y;
    kartesianY[3] = y;
    kartesianY[4] = sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(-135.0*(PI/180.0));
    kartesianY[5] = -sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(-45.0*(PI/180.0));
    # -------------z-------------
    kartesianZ[0] = z;
    kartesianZ[1] = z;
    kartesianZ[2] = z;
    kartesianZ[3] = z;
    kartesianZ[4] = z;
    kartesianZ[5] = z;
    # -------------------4. leg 1 (front left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[0]-35.74-20.15,2)+pow(kartesianY[0]-56-20.15,2))-32.25;
    a1 = asin((kartesianY[0]-56-20.15)/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[0],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[0]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[0] = a1*(180.0/PI);
    kartesianAngles[1] = 90-b2*(180.0/PI);
    kartesianAngles[2] = a2*(180.0/PI);
    # -------------------5. leg 2 (front right)-------
    x1 = sqrt(pow(-kartesianX[1]-35.74-20.15,2)+pow(kartesianY[1]-56-20.15,2))-32.25;
    a1 = asin((kartesianY[1]-56-20.15)/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[1],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[1]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[3] = a1*(180.0/PI);
    kartesianAngles[4] = 90-b2*(180.0/PI);
    kartesianAngles[5] = a2*(180.0/PI);
    # -------------------6. leg 3 (middle left)-------
    x1 = sqrt(pow(kartesianX[2]-37.5-28.5,2)+pow(kartesianY[2],2))-32.25;
    a1 = asin(kartesianY[2]/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[2],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[2]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[6] = a1*(180.0/PI);
    kartesianAngles[7] = 90-b2*(180.0/PI);
    kartesianAngles[8] = a2*(180.0/PI);
    # -------------------7. leg 4 (middle right)------
    x1 = sqrt(pow(-kartesianX[3]-37.5-28.5,2)+pow(kartesianY[3],2))-32.25;
    a1 = asin(kartesianY[3]/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[3],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[3]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[9] = a1*(180.0/PI);
    kartesianAngles[10] = 90-b2*(180.0/PI);
    kartesianAngles[11] = a2*(180.0/PI);
    # -------------------8. leg 5 (back left)---------
    x1 = sqrt(pow(kartesianX[4]-35.74-20.15,2)+pow(-kartesianY[4]-56-20.15,2))-32.25;
    a1 = asin((-kartesianY[4]-56-20.15)/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[4],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[4]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[12] = -a1*(180.0/PI);
    kartesianAngles[13] = 90-b2*(180.0/PI);
    kartesianAngles[14] = a2*(180.0/PI);
    # -------------------9. leg 6 (back right)--------
    x1 = sqrt(pow(-kartesianX[5]-35.74-20.15,2)+pow(-kartesianY[5]-56-20.15,2))-32.25;
    a1 = asin((-kartesianY[5]-56-20.15)/(x1+32.25));
    x2 = sqrt(pow(x1,2)+pow(kartesianZ[5],2));
    a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
    b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
    g1 = asin(kartesianZ[5]/x2);
    b2 = b1-g1;
    # save angles in array
    kartesianAngles[15] = -a1*(180.0/PI);
    kartesianAngles[16] = 90-b2*(180.0/PI);
    kartesianAngles[17] = a2*(180.0/PI);
    # put kartesian angles into array for plotting
    for i in range(0,len(all_angles),1):
        for j in range(0,len(all_angles[i]),1):
            all_angles[i][j] = kartesianAngles[i*len(all_angles[i])+j]*(PI/180.0)

# function to implement the updated translation inverse kinematics based on simulations
def inverse_kinematics_2():
    global all_angles
    global kartesianX
    global kartesianY
    global kartesianZ
    global kartesian
    # constants used in the calculations
    l1m = 32.25
    l1corr = 0.09
    l2 = 44
    l3 = 69.5
    # get requested coordinates and angles first
    x1 = defaultLegTransform[0]-kartesian[0];
    x2 = defaultLegTransform[0]+kartesian[0];
    y = defaultLegTransform[1]+kartesian[1];
    z = defaultLegTransform[2]+kartesian[2];
    #a = defaultLegTransform[3]+kartesian[3];
    #b = defaultLegTransform[4]+kartesian[4];
    #g = defaultLegTransform[5]+kartesian[5];
    
    # set supposed kartesian coordinates for all legs
    # -------------leg 1-------------
    atemp = np.arctan2(y,x1+l1m);
    kartesianX[0] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[0] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 2-------------
    atemp = np.arctan2(y,x2+l1m);
    kartesianX[1] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[1] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 3-------------
    kartesianX[2] = -(x1+l1m);
    kartesianY[2] = y;
    # -------------leg 4-------------
    kartesianX[3] = x2+l1m;
    kartesianY[3] = y;
    # -------------leg 5-------------
    atemp = np.arctan2(y,x1+l1m);
    kartesianX[4] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[4] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 6-------------
    atemp = np.arctan2(y,x2+l1m);
    kartesianX[5] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[5] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------z-------------
    kartesianZ[0] = z;
    kartesianZ[1] = z;
    kartesianZ[2] = z;
    kartesianZ[3] = z;
    kartesianZ[4] = z;
    kartesianZ[5] = z;
    # -------------------4. leg 1 (front left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[0],2)+pow(kartesianY[0],2));
    a1 = asin((kartesianY[0])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[0],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[0]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[0] = a1*(180.0/PI)-45;
    kartesianAngles[1] = b2*(180.0/PI);
    kartesianAngles[2] = 180-a2*(180.0/PI);
    # -------------------5. leg 2 (front right)-------
    x1 = sqrt(pow(kartesianX[1],2)+pow(kartesianY[1],2));
    a1 = asin((kartesianY[1])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[1],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[1]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[3] = a1*(180.0/PI)-45;
    kartesianAngles[4] = b2*(180.0/PI);
    kartesianAngles[5] = 180-a2*(180.0/PI);
    # -------------------6. leg 3 (middle left)-------
    x1 = sqrt(pow(kartesianX[2],2)+pow(kartesianY[2],2));
    a1 = asin(kartesianY[2]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[2],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[2])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[6] = a1*(180.0/PI);
    kartesianAngles[7] = b2*(180.0/PI);
    kartesianAngles[8] = 180-a2*(180.0/PI);
    # -------------------7. leg 4 (middle right)------
    x1 = sqrt(pow(kartesianX[3],2)+pow(kartesianY[3],2));
    a1 = asin(kartesianY[3]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[3],2));
    #a2 = np.radians(0)
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[3])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[9] = a1*(180.0/PI);
    kartesianAngles[10] = b2*(180.0/PI);
    kartesianAngles[11] = 180-a2*(180.0/PI);
    # -------------------8. leg 5 (back left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[4],2)+pow(kartesianY[4],2));
    a1 = asin((kartesianY[4])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[4],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[4]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[12] = a1*(180.0/PI)+45;
    kartesianAngles[13] = b2*(180.0/PI);
    kartesianAngles[14] = 180-a2*(180.0/PI);
    # -------------------9. leg 6 (back right)-------
    x1 = sqrt(pow(kartesianX[5],2)+pow(kartesianY[5],2));
    a1 = asin((kartesianY[5])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[5],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[5]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[15] = a1*(180.0/PI)+45;
    kartesianAngles[16] = b2*(180.0/PI);
    kartesianAngles[17] = 180-a2*(180.0/PI);
    
    # ------------------ angle transfer--------------
    # put kartesian angles into array for plotting
    for i in range(0,len(all_angles),1):
        for j in range(0,len(all_angles[i]),1):
            all_angles[i][j] = kartesianAngles[i*len(all_angles[i])+j]*(PI/180.0)

# function to implement the updated inverse kinematics based on simulations
def inverse_kinematics_3():
    global all_angles
    global kartesianX
    global kartesianY
    global kartesianZ
    global kartesian
    # constants used in the calculations
    y0 = 56+4.24
    x0 = 31.5
    x1 = 35.74
    l1m = 32.25
    l0m = 28.5
    l0o = 20.15
    l1corr = 0.09
    l2 = 44
    l3 = 69.5
    # get requested coordinates and angles first
    x1 = defaultLegTransform[0]+kartesian[0];
    x2 = defaultLegTransform[0]-kartesian[0];
    y = defaultLegTransform[1]-kartesian[1];
    z = defaultLegTransform[2]-kartesian[2];
    a = defaultLegTransform[3]+kartesian[3];
    b = defaultLegTransform[4]+kartesian[4];
    g = defaultLegTransform[5]+kartesian[5];
    
    # ONLY FOR PYTHON
    a = np.degrees(a)
    b = np.degrees(b)
    g = np.degrees(g)
    
    # set supposed kartesian coordinates for all legs
    # -------------leg 1-------------
    atemp = np.arctan2(y,x1+l1m);
    kartesianX[0] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[0] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 2-------------
    atemp = np.arctan2(y,x2+l1m);
    kartesianX[1] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[1] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 3-------------
    kartesianX[2] = -(x1+l1m);
    kartesianY[2] = y;
    # -------------leg 4-------------
    kartesianX[3] = x2+l1m;
    kartesianY[3] = y;
    # -------------leg 5-------------
    atemp = np.arctan2(y,x1+l1m);
    kartesianX[4] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[4] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------leg 6-------------
    atemp = np.arctan2(y,x2+l1m);
    kartesianX[5] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
    kartesianY[5] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
    # -------------z-------------
    kartesianZ[0] = z;
    kartesianZ[1] = z;
    kartesianZ[2] = z;
    kartesianZ[3] = z;
    kartesianZ[4] = z;
    kartesianZ[5] = z;
    # -------------------1. yaw-----------------------
    # front servos
    dy = (y0+l0o)*(1-cos(a*(PI/180.0)));
    dz = (y0+l0o)*sin(a*(PI/180.0));
    kartesianY[0] -= dy;
    kartesianZ[0] -= dz;
    kartesianY[1] -= dy;
    kartesianZ[1] -= dz;
    # back servos
    kartesianY[4] += dy;
    kartesianZ[4] += dz;
    kartesianY[5] += dy;
    kartesianZ[5] += dz;
    # -------------------2. pitch---------------------
    # front and servos
    dx = 2*(x1+l0o)*(1-cos(b*(PI/180.0)));
    dz = 2*(x1+l0o)*sin(b*(PI/180.0));
    kartesianX[0] += dx;
    kartesianZ[0] -= dz;
    kartesianX[1] -= dx;
    kartesianZ[1] += dz;
    kartesianX[4] += dx;
    kartesianZ[4] -= dz;
    kartesianX[5] -= dx;
    kartesianZ[5] += dz;
    # middle servos
    dx = 2*(x0+l0m)*(1-cos(b*(PI/180.0)));
    dz = 2*(x0+l0m)*sin(b*(PI/180.0));
    kartesianX[2] += dx;
    kartesianZ[2] -= dz;
    kartesianX[3] -= dx;
    kartesianZ[3] += dz;
    # -------------------3. roll----------------------
    # front servos
    g0 = atan((y0+l0o)/(x1+l0o))*(PI/180.0);
    dx = 1.25*sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
    dy = 1.25*sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
    kartesianX[0] += dx;
    kartesianY[0] += dy;
    kartesianX[1] += dx;
    kartesianY[1] -= dy;
    # middle servos
    comp_fac = 2.3666666666666667 # (l2+l3+l0m)/(x0+l0m)
    # COMP_FAC IS ONLY VALID FOR DEFAULT TRANSFORM
    dy = comp_fac*(x0+l0m)*sin((g)*(PI/180.0));
    dx = comp_fac*(x0+l0m)*(1-cos(g*(PI/180.0)));
    kartesianX[2] += dx;
    kartesianY[2] += dy;
    kartesianX[3] -= dx;
    kartesianY[3] -= dy;
    # back servos
    dx = sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
    dy = sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
    kartesianX[4] -= dx;
    kartesianY[4] += dy;
    kartesianX[5] -= dx;
    kartesianY[5] -= dy;
    # -------------------4. leg 1 (front left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[0],2)+pow(kartesianY[0],2));
    a1 = asin((kartesianY[0])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[0],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[0]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[0] = a1*(180.0/PI)-45;
    kartesianAngles[1] = b2*(180.0/PI);
    kartesianAngles[2] = 180-a2*(180.0/PI);
    # -------------------5. leg 2 (front right)-------
    x1 = sqrt(pow(kartesianX[1],2)+pow(kartesianY[1],2));
    a1 = asin((kartesianY[1])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[1],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[1]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[3] = a1*(180.0/PI)-45;
    kartesianAngles[4] = b2*(180.0/PI);
    kartesianAngles[5] = 180-a2*(180.0/PI);
    # -------------------6. leg 3 (middle left)-------
    x1 = sqrt(pow(kartesianX[2],2)+pow(kartesianY[2],2));
    a1 = asin(kartesianY[2]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[2],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[2])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[6] = a1*(180.0/PI);
    kartesianAngles[7] = b2*(180.0/PI);
    kartesianAngles[8] = 180-a2*(180.0/PI);
    # -------------------7. leg 4 (middle right)------
    x1 = sqrt(pow(kartesianX[3],2)+pow(kartesianY[3],2));
    a1 = asin(kartesianY[3]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[3],2));
    #a2 = np.radians(0)
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[3])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[9] = a1*(180.0/PI);
    kartesianAngles[10] = b2*(180.0/PI);
    kartesianAngles[11] = 180-a2*(180.0/PI);
    # -------------------8. leg 5 (back left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[4],2)+pow(kartesianY[4],2));
    a1 = asin((kartesianY[4])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[4],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[4]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[12] = a1*(180.0/PI)+45;
    kartesianAngles[13] = b2*(180.0/PI);
    kartesianAngles[14] = 180-a2*(180.0/PI);
    # -------------------9. leg 6 (back right)-------
    x1 = sqrt(pow(kartesianX[5],2)+pow(kartesianY[5],2));
    a1 = asin((kartesianY[5])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[5],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[5]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[15] = a1*(180.0/PI)+45;
    kartesianAngles[16] = b2*(180.0/PI);
    kartesianAngles[17] = 180-a2*(180.0/PI);
    
    # ------------------ angle transfer--------------
    # put kartesian angles into array for plotting
    for i in range(0,len(all_angles),1):
        for j in range(0,len(all_angles[i]),1):
            all_angles[i][j] = kartesianAngles[i*len(all_angles[i])+j]*(PI/180.0)

# function to implement the inverse kinematics for walking
def inverse_kinematics_4():
    global all_angles
    global kartesianX
    global kartesianY
    global kartesianZ
    global kartesian
    global is_walking
    # constants used in the calculations
    y0 = 56+4.24
    x0 = 31.5
    x1 = 35.74
    l1m = 32.25
    l0m = 28.5
    l0o = 20.15
    l1corr = 0.09
    l2 = 44
    l3 = 69.5
    # check if walking is disabled, otherwise kartesianXYZ are already set
    if not is_walking:
        # get requested coordinates and angles first
        x1 = defaultLegTransform[0]+kartesian[0];
        x2 = defaultLegTransform[0]-kartesian[0];
        y = defaultLegTransform[1]-kartesian[1];
        z = defaultLegTransform[2]-kartesian[2];
        a = defaultLegTransform[3]+kartesian[3];
        b = defaultLegTransform[4]+kartesian[4];
        g = defaultLegTransform[5]+kartesian[5];
        # ONLY FOR PYTHON
        a = np.degrees(a)
        b = np.degrees(b)
        g = np.degrees(g)
        # set supposed kartesian coordinates for all legs
        # -------------leg 1-------------
        atemp = np.arctan2(y,x1+l1m);
        kartesianX[0] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
        kartesianY[0] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
        # -------------leg 2-------------
        atemp = np.arctan2(y,x2+l1m);
        kartesianX[1] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
        kartesianY[1] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
        # -------------leg 3-------------
        kartesianX[2] = -(x1+l1m);
        kartesianY[2] = y;
        # -------------leg 4-------------
        kartesianX[3] = x2+l1m;
        kartesianY[3] = y;
        # -------------leg 5-------------
        atemp = np.arctan2(y,x1+l1m);
        kartesianX[4] = -(sqrt(pow(x1+l1m,2)+pow(y,2)))*cos(atemp);
        kartesianY[4] = (sqrt(pow(x1+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
        # -------------leg 6-------------
        atemp = np.arctan2(y,x2+l1m);
        kartesianX[5] = (sqrt(pow(x2+l1m,2)+pow(y,2)))*cos(atemp);
        kartesianY[5] = (sqrt(pow(x2+l1m+l1corr,2)+pow(y,2)))*sin(atemp);
        # -------------z-------------
        kartesianZ[0] = z;
        kartesianZ[1] = z;
        kartesianZ[2] = z;
        kartesianZ[3] = z;
        kartesianZ[4] = z;
        kartesianZ[5] = z;
        # -------------------1. yaw-----------------------
        # front servos
        dy = (y0+l0o)*(1-cos(a*(PI/180.0)));
        dz = (y0+l0o)*sin(a*(PI/180.0));
        kartesianY[0] -= dy;
        kartesianZ[0] -= dz;
        kartesianY[1] -= dy;
        kartesianZ[1] -= dz;
        # back servos
        kartesianY[4] += dy;
        kartesianZ[4] += dz;
        kartesianY[5] += dy;
        kartesianZ[5] += dz;
        # -------------------2. pitch---------------------
        # front and back servos
        dx = 2*(x1+l0o)*(1-cos(b*(PI/180.0)));
        dz = 2*(x1+l0o)*sin(b*(PI/180.0));
        kartesianX[0] += dx;
        kartesianZ[0] -= dz;
        kartesianX[1] -= dx;
        kartesianZ[1] += dz;
        kartesianX[4] += dx;
        kartesianZ[4] -= dz;
        kartesianX[5] -= dx;
        kartesianZ[5] += dz;
        # middle servos
        dx = 2*(x0+l0m)*(1-cos(b*(PI/180.0)));
        dz = 2*(x0+l0m)*sin(b*(PI/180.0));
        kartesianX[2] += dx;
        kartesianZ[2] -= dz;
        kartesianX[3] -= dx;
        kartesianZ[3] += dz;
        # -------------------3. roll----------------------
        # front servos
        g0 = atan((y0+l0o)/(x1+l0o))*(PI/180.0);
        dx = 1.25*sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
        dy = 1.25*sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
        kartesianX[0] += dx;
        kartesianY[0] += dy;
        kartesianX[1] += dx;
        kartesianY[1] -= dy;
        # middle servos
        comp_fac = (l2+l3+l0m)/(x0+l0m)
        # COMP_FAC IS ONLY VALID FOR DEFAULT TRANSFORM
        dy = comp_fac*(x0+l0m)*sin((g)*(PI/180.0));
        dx = comp_fac*(x0+l0m)*(1-cos(g*(PI/180.0)));
        kartesianX[2] += dx;
        kartesianY[2] += dy;
        kartesianX[3] -= dx;
        kartesianY[3] -= dy;
        # back servos
        dx = sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
        dy = sqrt(pow(y0+l0o,2)+pow(x1+l0o,2))*sin((g0+g)*(PI/180.0));
        kartesianX[4] -= dx;
        kartesianY[4] += dy;
        kartesianX[5] -= dx;
        kartesianY[5] -= dy;
    # -------------------4. leg 1 (front left)--------
    # calculate angles
    x1 = sqrt(pow(kartesianX[0],2)+pow(kartesianY[0],2));
    a1 = asin((kartesianY[0])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[0],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[0]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[0] = a1*(180.0/PI)-45;
    kartesianAngles[1] = b2*(180.0/PI);
    kartesianAngles[2] = 180-a2*(180.0/PI);
    # -------------------5. leg 2 (front right)-------
    x1 = sqrt(pow(kartesianX[1],2)+pow(kartesianY[1],2));
    a1 = asin((kartesianY[1])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[1],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[1]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[3] = a1*(180.0/PI)-45;
    kartesianAngles[4] = b2*(180.0/PI);
    kartesianAngles[5] = 180-a2*(180.0/PI);
    # -------------------6. leg 3 (middle left)-------
    x1 = sqrt(pow(kartesianX[2],2)+pow(kartesianY[2],2));
    a1 = asin(kartesianY[2]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[2],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[2])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[6] = a1*(180.0/PI);
    kartesianAngles[7] = b2*(180.0/PI);
    kartesianAngles[8] = 180-a2*(180.0/PI);
    # -------------------7. leg 4 (middle right)------
    x1 = sqrt(pow(kartesianX[3],2)+pow(kartesianY[3],2));
    a1 = asin(kartesianY[3]/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[3],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin((kartesianZ[3])/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[9] = a1*(180.0/PI);
    kartesianAngles[10] = b2*(180.0/PI);
    kartesianAngles[11] = 180-a2*(180.0/PI);
    # -------------------8. leg 5 (back left)--------
    x1 = sqrt(pow(kartesianX[4],2)+pow(kartesianY[4],2));
    a1 = asin((kartesianY[4])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[4],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[4]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[12] = a1*(180.0/PI)+45;
    kartesianAngles[13] = b2*(180.0/PI);
    kartesianAngles[14] = 180-a2*(180.0/PI);
    # -------------------9. leg 6 (back right)-------
    x1 = sqrt(pow(kartesianX[5],2)+pow(kartesianY[5],2));
    a1 = asin((kartesianY[5])/(x1));
    x2 = sqrt(pow(x1-l1m,2)+pow(kartesianZ[5],2));
    a2 = acos((pow(l2,2)+pow(l3,2)-pow(x2,2))/(2*l2*l3));
    b1 = acos((pow(l2,2)+pow(x2,2)-pow(l3,2))/(2*l2*x2));
    g1 = asin(kartesianZ[5]/x2);
    b2 = b1+g1;
    # save angles in array
    kartesianAngles[15] = a1*(180.0/PI)+45;
    kartesianAngles[16] = b2*(180.0/PI);
    kartesianAngles[17] = 180-a2*(180.0/PI);
    
    # ------------------ angle transfer--------------
    # put kartesian angles into array for plotting
    for i in range(0,len(all_angles),1):
        for j in range(0,len(all_angles[i]),1):
            all_angles[i][j] = kartesianAngles[i*len(all_angles[i])+j]*(PI/180.0)

if mode:
    # calculate inverse kinematics once
    # inverse_kinematics_1()
    # inverse_kinematics_2()
    # inverse_kinematics_3()
    inverse_kinematics_4()
    # calculate reference positions
    predefine_positions()
# update transforms once
update_transforms()

# open plot
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(19.2, 10.8)
# create subplot
ax = plt.subplot(projection="3d")
plt.subplots_adjust(left=0.15, bottom=0.25)
# draw all things once
initial_draw()
if mode:
    # create sliders
    slider1_ax = plt.axes([0.2, 0.20, 0.65, 0.03])
    slider2_ax = plt.axes([0.2, 0.17, 0.65, 0.03])
    slider3_ax = plt.axes([0.2, 0.14, 0.65, 0.03])
    slider4_ax = plt.axes([0.2, 0.11, 0.65, 0.03])
    slider5_ax = plt.axes([0.2, 0.08, 0.65, 0.03])
    slider6_ax = plt.axes([0.2, 0.05, 0.65, 0.03])
    # initialize sliders
    slider1 = Slider(slider1_ax, 'X', -100, 100, valinit=0)
    slider2 = Slider(slider2_ax, 'Y', -100, 100, valinit=0)
    slider3 = Slider(slider3_ax, 'Z', -100, 100, valinit=0)
    slider4 = Slider(slider4_ax, 'Rx', -45, 45, valinit=0)
    slider5 = Slider(slider5_ax, 'Ry', -45, 45, valinit=0)
    slider6 = Slider(slider6_ax, 'Rz', -45, 45, valinit=0)
    # set method for updates
    slider1.on_changed(update_slider_xyz)
    slider2.on_changed(update_slider_xyz)
    slider3.on_changed(update_slider_xyz)
    slider4.on_changed(update_slider_xyz)
    slider5.on_changed(update_slider_xyz)
    slider6.on_changed(update_slider_xyz)
else:
    # create sliders
    slider1_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
    slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
    slider3_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
    # initialize sliders
    slider1 = Slider(slider1_ax, 'Hip', -90, 90, valinit=0)
    slider2 = Slider(slider2_ax, 'Knee', -90, 90, valinit=0)
    slider3 = Slider(slider3_ax, 'Foot', -45, 135, valinit=0)
    # set method for updates
    slider1.on_changed(update_slider_angle)
    slider2.on_changed(update_slider_angle)
    slider3.on_changed(update_slider_angle)
    
animation_1 = animation.FuncAnimation(plt.gcf(),animate,interval=50)
#plt.tight_layout()
plt.show()
