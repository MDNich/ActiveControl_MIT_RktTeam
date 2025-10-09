# Import necessary libraries.
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.optimize as spopt

# For LaTeX-powered plotting:
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['text.usetex'] = True
rcParams['font.size'] = 14


freq = 500 # Hz
datGyro = np.array(pd.read_csv('gyro_out.txt', header = 2))
datGyro = np.array(pd.read_csv('magnet_out.txt', header = 2))
timeseries = np.arange(0,len(datGyro)/freq,1/freq)

endTS = timeseries#[timeseries > 20]
endGyr = datGyro#[timeseries > 20]

timeIndex = 0

#procGyr = []
#for i in range(len(endTS)-10):
#	procGyr.append(np.mean(endGyr[i:i+10]))

plt.plot(endTS,endGyr)
plt.show()