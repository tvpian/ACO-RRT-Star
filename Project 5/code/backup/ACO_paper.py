# Ant Colony optimzation for continuous domains

import numpy as np
import matplotlib.pyplot as plt

# Ants are stored in a table 
# Each row contains one of the k ants
# d is the number of dimensions of the problem


# Gaussian kernel probability density function for sampling next location for k ants
def gaussian_kernel(x, mu_l, sigma_l, w_l):

    return w_l * (1 / (sigma_l * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu_l) / sigma_l) ** 2)

