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
datGyro = np.array(pd.read_csv('gyro_out.txt', header = 1))
print(datGyro)
timeseries = np.arange(0,len(datGyro)/freq,1/freq)

integrateArrElementwise = lambda arr,n: np.sum(arr[0:n])/freq

integratedGyr = []
for i in range(len(timeseries)):
	integratedGyr.append(integrateArrElementwise(datGyro,i))

timeIndex = 0

procGyr = []
for i in range(len(timeseries)-250):
	procGyr.append(np.mean(datGyro[i:i+250]))

#plt.plot(timeseries,integratedGyr)
#plt.plot(timeseries,datGyro)
plt.plot(timeseries[250:],procGyr)
plt.show()