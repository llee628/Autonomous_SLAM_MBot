import sys
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib.animation as animation
sys.path.append("lcmtypes")
import lcm
from lcmtypes import particles_t
#from lcmtypes import particle_t
from lcmtypes import pose_xyt_t
from time import sleep

if len(sys.argv) < 2:
    sys.stderr.write("usage: mbot_controller_plot.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

channels_of_particles_dict = {}
particles_dict_default = {'weights':[], 'x':[], 'y':[], 'theta':[]}


sample_x = []
sample_y = []
sample_theta = []

action_x = []
action_y = []
action_theta = []

sensor_x = []
sensor_y = []
sensor_theta = []

sample_weights = []
action_weights = []
sensor_weights = []

truth_x = 0
truth_y = 0
truth_theta = 0

percent_widen_graphs = 10.0
min_x = 0
max_x = 0
min_y = 0
max_y = 0
min_theta = 0
max_theta = 0


# SET MIN/MAX VALUES
for event in log:
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)
        x_val = msg.x
        y_val = msg.y
        theta_val = msg.theta
        
        if x_val < min_x: min_x = x_val * (1 - (percent_widen_graphs/200)) 
        if x_val > max_x: max_x = x_val * (1 + (percent_widen_graphs/200))

        if y_val < min_y: min_y = y_val * (1 - (percent_widen_graphs/200))
        if y_val > max_y: max_y = y_val * (1 + (percent_widen_graphs/200))
        
        if theta_val < min_theta: min_theta = theta_val * (1 - (percent_widen_graphs/200))
        if theta_val > max_theta: max_theta = theta_val * (1 + (percent_widen_graphs/200))


for event in log:
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)

        truth_x = (msg.x)
        truth_y = (msg.y)
        truth_theta = (msg.theta)
    
    if event.channel.find('PARTICLES'):
        channels_of_particles_dict[event.channel] = particles_dict_default
        for i in range(msg.num_particles):
            channels_of_particles_dict[event.channel]['weights'] = msg.particles[i].weight
            channels_of_particles_dict[event.channel]['x'] = msg.particles[i].pose.x
            channels_of_particles_dict[event.channel]['y'] = msg.particles[i].pose.y
            channels_of_particles_dict[event.channel]['theta'] = msg.particles[i].pose.theta

{
    # if event.channel == "SAMPLE_PARTICLES":
    #     msg = particles_t.decode(event.data)
    #     particles_dict[event.channel]

    #     weights_new = []
    #     particles = msg.particles
    #     x_new = []
    #     y_new = []
    #     theta_new = []
    #     for i in range(msg.num_particles):

    #         weights_new.append(msg.particles[i].weight)

    #         x_new.append(msg.particles[i].pose.x)
    #         y_new.append(msg.particles[i].pose.y)
    #         theta_new.append(msg.particles[i].pose.theta)

    #     sample_weights = weights_new
    #     sample_x = x_new
    #     sample_y = y_new
    #     sample_theta = theta_new

    # if event.channel == "ACTION_PARTICLES":
    #     msg = particles_t.decode(event.data)
    #     weights_new = []
    #     particles = msg.particles
    #     x_new = []
    #     y_new = []
    #     theta_new = []
    #     for i in range(msg.num_particles):

    #         weights_new.append(msg.particles[i].weight)

    #         x_new.append(msg.particles[i].pose.x)
    #         y_new.append(msg.particles[i].pose.y)
    #         theta_new.append(msg.particles[i].pose.theta)

    #     action_weights = weights_new
    #     action_x = x_new
    #     action_y = y_new
    #     action_theta = theta_new    

    # if event.channel == "SENSOR_PARTICLES":
    #     msg = particles_t.decode(event.data)
    #     weights_new = []
    #     particles = msg.particles
    #     x_new = []
    #     y_new = []
    #     theta_new = []
    #     for i in range(msg.num_particles):

    #         weights_new.append(msg.particles[i].weight)

    #         x_new.append(msg.particles[i].pose.x)
    #         y_new.append(msg.particles[i].pose.y)
    #         theta_new.append(msg.particles[i].pose.theta)

    #     sensor_weights = weights_new
    #     sensor_x = x_new
    #     sensor_y = y_new
    #     sensor_theta = theta_new
}
        subplot_matrix_index = len(channels_of_particles_dict)*100 + 31

        for channel_name, particles in channels_of_particles_dict.items():
            plt.subplot(subplot_matrix_index)
            plt.ylabel(channel_name)
            plt.hist(particles['x'], weights=particles['weights'], label='slam', color='b')
            plt.xlabel("X")
            plt.xlim([min_x,max_x])
            plt.axvline(truth_x, color='r', linestyle='dashed', linewidth=1)

            plt.subplot(subplot_matrix_index + 1)
            # plt.title('RESAMPLED PARTICLE WEIGHTS')
            plt.hist(sample_particles['y'], weights=particles['weights'],label='slam', color='b')
            plt.xlabel("Y")
            plt.xlim([min_y,max_y])
            plt.axvline(truth_y, color='r', linestyle='dashed', linewidth=1)
            
            plt.subplot(subplot_matrix_index + 2)
            plt.hist(particles['theta'], weights=particles['weights'],label='slam', color='b')
            plt.xlabel("THETA")
            plt.xlim([min_theta,max_theta])
            plt.axvline(truth_theta, color='r', linestyle='dashed', linewidth=1)
            
            subplot_matrix_index = subplot_matrix_index + 3

{
        # ########################################################################################
        # plt.subplot(331)
        # plt.ylabel("RESAMPLED WEIGHT")
        # plt.hist(sample_x, weights=sample_weights,label='slam', color='b')
        # plt.xlabel("X")
        # plt.xlim([min_x,max_x])
        # plt.axvline(truth_x, color='r', linestyle='dashed', linewidth=1)

        # plt.subplot(332)
        # # plt.title('RESAMPLED PARTICLE WEIGHTS')
        # plt.hist(sample_y, weights=sample_weights,label='slam', color='b')
        # plt.xlabel("Y")
        # plt.xlim([min_y,max_y])
        # plt.axvline(truth_y, color='r', linestyle='dashed', linewidth=1)
        
        # plt.subplot(333)
        # plt.hist(sample_theta, weights=sample_weights,label='slam', color='b')
        # plt.xlabel("THETA")
        # plt.xlim([min_theta,max_theta])
        # plt.axvline(truth_theta, color='r', linestyle='dashed', linewidth=1)
        # ########################################################################################
        # plt.subplot(334)
        # plt.ylabel("ACTION WEIGHT")
        # plt.hist(action_x, weights=action_weights,label='slam', color='g')
        # plt.xlabel("X")
        # plt.xlim([min_x,max_x])
        # plt.axvline(truth_x, color='r', linestyle='dashed', linewidth=1)

        # plt.subplot(335)
        # # plt.title('ACTION MODEL PARTICLE WEIGHTS')
        # plt.hist(action_y, weights=action_weights,label='slam', color='g')
        # plt.xlabel("Y")
        # plt.xlim([min_y,max_y])
        # plt.axvline(truth_y, color='r', linestyle='dashed', linewidth=1)
        
        # plt.subplot(336)
        # plt.hist(action_theta, weights=action_weights,label='slam', color='g')
        # plt.xlabel("THETA")
        # plt.xlim([min_theta,max_theta])
        # plt.axvline(truth_theta, color='r', linestyle='dashed', linewidth=1)
        # ########################################################################################
        # plt.subplot(337)
        # plt.ylabel("SENSOR WEIGHT")
        # plt.hist(sensor_x, weights=sensor_weights,label='slam', color='m')
        # plt.xlabel("X")
        # plt.xlim([min_x,max_x])
        # plt.axvline(truth_x, color='r', linestyle='dashed', linewidth=1)

        # plt.subplot(338)
        # # plt.title('SENSOR MODEL PARTICLE WEIGHTS')
        # plt.hist(sensor_y, weights=sensor_weights,label='slam', color='m')
        # plt.xlabel("Y")
        # plt.xlim([min_y,max_y])
        # plt.axvline(truth_y, color='r', linestyle='dashed', linewidth=1)
        
        # plt.subplot(339)
        # plt.hist(sensor_theta, weights=sensor_weights,label='slam', color='m')
        # plt.xlabel("THETA")
        # plt.xlim([min_theta,max_theta])
        # plt.axvline(truth_theta, color='r', linestyle='dashed', linewidth=1)
        # ########################################################################################
}
        plt.pause(0.0000005)
        plt.clf()