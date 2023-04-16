# -*- coding: utf-8 -*-
"""
Created on Sun Feb 19 10:31:43 2023

https://stackoverflow.com/questions/42281966/how-to-plot-vectors-in-python-using-matplotlib#42283226

@author: Philipp Schulz
"""

import numpy as np
import matplotlib.pyplot as plt

V = np.array([[1,1], [-2,2], [4,-7]])
origin = np.array([[0, 0, 0],[0, 0, 0]]) # origin point

plt.quiver(*origin, V[:,0], V[:,1], color=['r','b','g'], angles='xy', scale_units='xy', scale=200)
v12 = V[0] + V[1] # adding up the 1st (red) and 2nd (blue) vectors
plt.quiver(*origin, v12[0], v12[1])
plt.show()

