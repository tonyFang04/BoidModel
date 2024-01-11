import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

noOfParticles = 400
timeStep = 1000
fig = plt.figure()
ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))
particles = np.vsplit(np.flipud(np.rot90(pd.read_csv("data.csv",header=None))),noOfParticles)

trajectories = [ax.plot([], [],color='blue',markersize=1.5,marker='o',alpha=1,animated=True)[0] for i in np.arange(noOfParticles)]

def init():
    for trajectory in trajectories:
        trajectory.set_data([],[])
    return trajectories

def animate(frames):
    for trajectory, particle in zip(trajectories, particles):
        trajectory.set_data(particle[1, frames-1:frames],particle[0, frames-1:frames])
    return trajectories

animacion = animation.FuncAnimation(fig, animate, init_func=init, frames=timeStep, interval=230, blit=True)
plt.show()
