#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

x = np.arange(80)
theta = np.linspace(0, 2 * np.pi, 80)  # theta is now used to calculate the sine

fig, ax = plt.subplots()

def update(i):
    y = np.sin(theta + np.random.uniform(0, 2*np.pi))
    ax.scatter(x, y, s=10) 

ani = FuncAnimation(fig, update, frames=10, repeat=False)

plt.show()
