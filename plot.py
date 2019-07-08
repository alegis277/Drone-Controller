import matplotlib.pyplot as plt
import numpy as np
import time
import pandas as pd

data = pd.read_csv("debug.csv")
time = np.array(data['Time (s)'])
height = np.array(data['Height (cm)'])
controlAction = np.array(data['Control Action (1000-2000)'])


plt.rcParams["figure.figsize"] = (14,7)
plt.rcParams["figure.subplot.hspace"] = 0.36
f, axes = plt.subplots(2,1)

axes[0].grid(b=True, which='major', color='#AAAAAA', linestyle='-')
axes[1].grid(b=True, which='major', color='#AAAAAA', linestyle='-')


axes[0].plot(time, height, c='darkmagenta')
axes[1].plot(time, controlAction, c='midnightblue')

axes[0].set_title("Height (cm)", size=15)
axes[1].set_title("Control Action", size=15)

axes[0].set_xlabel('Time (s)')
axes[1].set_xlabel('Time (s)')

axes[0].set_ylabel('Height (cm)')
axes[1].set_ylabel('Control Action (1000-2000)')

axes[0].set_ylim((0,200))
axes[1].set_ylim((1000,2000))

plt.show()





