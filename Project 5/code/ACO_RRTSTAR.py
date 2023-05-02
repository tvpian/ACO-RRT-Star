###Ant Colony Optimization for RRT* Path Planning###

'''
The algorithm consists of five steps. 
First,we initialize the ants that will generate future samples.
Second, we sample from the distribution described by the
current ants. 
Then we update our tree according to the original
RRT/RRT* algorithm. 
After that, we calculate the utility
of that sample based on how much it could improve the
current path. This is divided into two factors: (i) exploitation
of the current solution and (ii) exploration of the state space
to find a new, better solution. 
Based on that utility, we
update the ants and resample according to the new distribution.
'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import random
import time
import copy
import sys
import os
import pickle
import heapq
from scipy.spatial.distance import euclidean



# Utility class to store the utility of a sample
class Utility:
    def __init__(self, eta, d):
        # To store the exploration utility
        self.u_exploration = None
        # To store the exploitation utility
        self.u_exploitation = None
        # To store the tradeoff factor
        self.alpha = None
        # Parameters for exploration utility
        self.R = None
        self.eta = eta
        self.d = d
    
    # To calculate the total utility
    def calculate_utility(self):
        return self.alpha*self.u_exploitation + (1-self.alpha)*self.u_exploration

    def calculate_u_exploitation(self, c_max, c_path, cost_euclidean_x_xb, cost_graph_xb, cost_euclidean_xa_xb, state):
        if state == 1:
            self.u_exploitation = 1 - (cost_euclidean_x_xb/c_max)
        elif state == 2:
            self.u_exploitation =   (c_path  - cost_graph_xb)/(c_path - cost_euclidean_xa_xb)
        elif state == 3:
            self.u_exploitation = cost_euclidean_xa_xb/(cost_graph_xb + cost_euclidean_x_xb)

        return self.u_exploitation

    def calculate_u_exploration(self, nodes, near_nodes):
        gamma = 1
        self.R = min(gamma * (np.log(len(nodes))/len(nodes))**(1 / self.d) , self.eta)
        c = 1  if len(near_nodes) == 0 else 1/len(near_nodes)
        self.u_exploration = 1/c * (self.R/self.eta)**self.d
        


        return self.u_exploration


class Table:
    def __init__(self, num_ants):
        self.num_ants = num_ants
        # To store the samples
        self.S = []
        # To store the utilities
        self.U = []
        # To store the probabilities
        self.W = []
 
    # Sample Ants from the table
    def sample_ant(self):
        P = []
        # Sample an ant from the table with probability W
        for i in range(len(self.S)):
            p = self.W[i]/sum(self.W)
            P.append(p)

        # Select an ant based on roulette wheel selection
        index = np.random.choice(len(self.S), p=P)
        # Compute the standard deviation of the chosen ant with respect to the other ants
        std_x = np.std(np.array(self.S)[:, 0])
        std_y = np.std(np.array(self.S)[:, 1])

        # Randomly sample from the distribution where mean = self.S[index] and std = [std_x, std_y]
        x_new = np.random.normal(self.S[index][0], std_x)
        y_new = np.random.normal(self.S[index][1], std_y)

        # print("x_new: ", x_new)
        # print("y_new: ", y_new)

        return np.array([x_new, y_new]), index

    def initialize_ants(self):
        width = 10
        height = 10
        # Store min and max of the horizontal and vertical axis of the map
        horizontal_map = [0, width]
        vertical_map = [0, height]

        search_map = np.zeros((width, height))

        for i in range(self.num_ants):
            # Sample a random point(x,y) from the obstacle free state space of specified dimension using Gaussian Kernal Probability Density Function
            x = np.random.uniform(low=horizontal_map[0], high=horizontal_map[1])
            y = np.random.uniform(low=vertical_map[0], high=vertical_map[1])
            ant = Ant(np.array([x,y]), 0)
            # Append the ant to the table
            self.S.append(ant.S)
            # Append the utility to the table
            self.U.append(ant.U)
            # Append the probability to the table
            self.W.append(1/self.num_ants)

    def update_ants(self, P):
        u_exploration, u_exploitation, new_node, end, alpha, l, path, c_path = P

        if c_path is math.inf or path is None:
            self.U = [None * self.num_ants]
            return
        
        # self.U[l] = 
        
        

class Ant:
    def __init__(self, position, utility):
        # To store random sample from obstacle free state space
        self.S = position
        # To store the utility of the sample that gives the importance of the sample
        # # Utility is defined as per the algorithm's optimization objective
        self.U = utility


        



if __name__ == "__main__":
    print ("ACO-RRT* Path Planning Algorithm")
    # Define a table T to store the samples and their utilities
    T = Table()
    # Define the number of ants
    k = 10
    # Define the number of iterations
    num_iterations = 100

    # Define a map of width and height 200 units
    width = 200
    height = 200
    # Store min and max of the horizontal and vertical axis of the map
    horizontal_map = [0, width]
    vertical_map = [0, height]

    search_map = np.zeros((width, height))

    # Randomly initialize the ants from the obstacle free state space
    for i in range(k):
        # Sample a random point(x,y) from the obstacle free state space of specified dimension using Gaussian Kernal Probability Density Function
        x = np.random.uniform(low=horizontal_map[0], high=horizontal_map[1])
        y = np.random.uniform(low=vertical_map[0], high=vertical_map[1])
        ant = Ant(np.array([x,y]), 0)
        # Append the ant to the table
        T.S.append(ant.S)
        # Append the utility to the table
        T.U.append(ant.U)
        # Append the probability to the table
        T.W.append(1/k)


    # Print the table
    print ("Table: ")
    print ("S: ", T.S)
    print ("U: ", T.U)
    print ("W: ", T.W)




