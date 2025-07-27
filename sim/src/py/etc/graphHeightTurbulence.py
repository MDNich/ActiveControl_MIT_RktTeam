import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy.optimize as spopt

def line(x,a,b):
	return x*a + b


turbs = np.arange(0,21,2.5)
turbDatArr = []
maxAlts = []
maxVels = []
finCantNoise = []
angVelNoise = []
maxAngVel = []
for turb in turbs:
	if turb%1 == 0.0:
		turb = int(turb)
	turbDatArr.append(pd.read_csv('../../../dat/csv/run_turb{}.csv'.format(turb), sep=','))	
	maxAlts.append(np.max(turbDatArr[-1]["Altitude (m)"]))
	maxVels.append(np.max(turbDatArr[-1]["Velocity (m/s)"]))
	finCantNoise.append(np.std(turbDatArr[-1]["Fin Cant Angle (deg)"]))
	angVelNoise.append(np.std(turbDatArr[-1]["Angular Velocity (rad/s)"]))
	maxAngVel.append(np.max(turbDatArr[-1]["Angular Velocity (rad/s)"]))


def quad(x,a,b,c):
	return x*x*a + b*x + c




from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True
fig,axs = plt.subplots(nrows=2,sharex='col')
ax1,ax2 = tuple(axs)
ax1.plot(turbs,maxAlts,c='b')
ax1.scatter(turbs,maxAlts,c='b')
ax1.set_ylabel("Maximum Altitude (m)")
ax11 = ax1.twinx()
ax11.plot(turbs,maxVels,c='r')
ax11.scatter(turbs,maxVels,c='r')
ax11.set_ylabel("Maximum Velocity (m/s)")
# 
ax2.plot(turbs,finCantNoise,c='b')
ax2.scatter(turbs,finCantNoise,c='b')
ax2.set_ylabel("Fin Cant Noise (deg)")
ax21 = ax2.twinx()
ax21.plot(turbs,angVelNoise,c='r')
ax21.scatter(turbs,angVelNoise,c='r')
ax21.set_ylabel("Angular Velocity Noise (rad/s)")
ax2.set_xlabel("Turbulence Percentage")



ax11.spines['right'].set_color('b')
ax11.spines['left'].set_color('r')
ax11.yaxis.label.set_color('b')
ax1.yaxis.label.set_color('r')
ax1.tick_params(axis='y', colors='red')
ax11.tick_params(axis='y', colors='b')
ax21.spines['right'].set_color('b')
ax21.spines['left'].set_color('r')
ax21.yaxis.label.set_color('b')
ax2.yaxis.label.set_color('r')
ax2.tick_params(axis='y', colors='red')
ax21.tick_params(axis='y', colors='b')
ax1.set_title("\\sc Variance of Launch Profile with Turbulence",pad=15)
plt.show()


fig,ax = plt.subplots()
#ax.plot(finCantNoise,maxAlts,c='b')
ax.scatter(finCantNoise,maxAlts,c='b')
ax2 = ax.twinx()
#ax2.plot(finCantNoise,maxVels,c='r')
ax2.scatter(finCantNoise,maxVels,c='r')
ax.set_xlabel('Fin Cant Noise (deg)')
ax.set_xlim(0,10)
ax2.tick_params(axis='y', colors='red')
ax.tick_params(axis='y', colors='b')
ax2.spines['right'].set_color('r')
ax2.spines['left'].set_color('b')
ax.yaxis.label.set_color('b')
ax2.yaxis.label.set_color('r')

lineRed,pcovRed = spopt.curve_fit(line,finCantNoise,maxVels)
lineBlue,pcovBlue = spopt.curve_fit(line,finCantNoise,maxAlts)

ax2.plot([0,10],[line(0,*lineRed),line(10,*lineRed)],c='r',label='$A(\\sigma_{\\rm fc})$:' + ' $a={}$, $b={}$'.format(*np.round(list(lineRed),2)))
ax.plot([0,10],[line(0,*lineBlue),line(10,*lineBlue)],c='b',label='$V_{\\rm max}(\\sigma_{\\rm fc})$:' + ' $a={}$, $b={}$'.format(*np.round(list(lineBlue),2)))
ax.legend(loc='upper left')
ax2.legend(loc='upper right')
ax.set_ylabel('Maxmium Altitude (m)')
ax.set_ylim(100,500)
ax2.set_ylabel('Maxmium Velocity (m/s)')
ax2.set_ylim(50,100)
ax.set_title("\\sc Regression of Altitude and Velocity with Varying Fin Cant Angle Distributions")

plt.show()

