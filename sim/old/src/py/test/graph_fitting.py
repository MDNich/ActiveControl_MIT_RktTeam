import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as spopt
import scipy.special as spspec


from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True


# import csv from
# ../../../dat/zephy_testlaunch/csv/zephyrus_testlaunch_OR_output_noAirbrakes.csv

dat = np.loadtxt('../../../dat/zephy_testlaunch/csv/zephyrus_testlaunch_OR_output_noAirbrakes.csv', delimiter=',',skiprows=1)
t = dat[:,0]
alt = dat[:,1]
vel = dat[:,2]
velMag = dat[:,3]
accelZ = dat[:,4]

g = 9.81


def vel_from_accel(t,alpha,t_apog):
    return -g*(t-t_apog) + alpha*((t -t_apog)**2)/2

def vel_fit(t, a, b, t_apog):
    return a*(t-t_apog)**3 + b*(t-t_apog)**2  - g*(t - t_apog)

def vel_fit_hardcoded_t_apog(t, a, b):
    global shared_t_apog
    return a*(t-shared_t_apog)**3 + b*(t-shared_t_apog)**2  - g*(t - shared_t_apog)


def altitude_model(t, a, b, t_apog, x0):
    return a*(t-t_apog)**4/4 + b*(t-t_apog)**3/3  - g*(t - t_apog)**2/2+x0

def accel_model(t, a, b, t_apog):
    return 3*a*(t-t_apog)**2 + 2*b*(t-t_apog) - g


def accel_quintic(t,a):
    global shared_t_apog
    return a*(t-shared_t_apog)**5 - g


def R2(func,y_coords,x_coords,*opt0):
    residuals = y_coords - np.round(func(x_coords, *opt0),3)
    ss_res = np.sum(residuals**2)
    ss_tot = np.sum((y_coords-np.mean(y_coords))**2)
    correlation = 1 - (ss_res / ss_tot)
    return correlation



index0 = np.argmin((vel-400)**2)
index1 = np.argmin((t-12)**2)
fit_T = t[index0:index1][::200][:13]
fit_V = vel[index0:index1][::200][:13]
fit_A = accelZ[index0:index1][::200][:13]
print("Fitting a timeseries of {} points.".format(len(fit_V)))



# shared_t_apog_A = 33.375


global shared_t_apog
    
shared_t_apogs = [33,34,35]
R2s = []
optsA = []
print("quintic fit for accel")
for shared_t_apog in shared_t_apogs:
    opt_Q_A,pcov = spopt.curve_fit(accel_quintic,fit_T,fit_A,maxfev=10000000)
    R2_Q_A = R2(accel_quintic,fit_A,fit_T,*opt_Q_A)
    print("Got fit with R2 {} for t_apog ".format(np.round(R2_Q_A,5), shared_t_apog))
    R2s.append(R2_Q_A)
    optsA.append(opt_Q_A)

best_t_apog = shared_t_apogs[np.argmax(R2s)]
opt_A_best = optsA[np.argmax(R2s)]

shared_t_apog = best_t_apog + 1.5

optA,pcovA = spopt.curve_fit(vel_fit_hardcoded_t_apog, fit_T, fit_V, maxfev=1000000, 
    p0=(5, 5),bounds=([-np.inf,-np.inf],[np.inf,np.inf]))
a,b = optA
R2_A = R2(vel_fit_hardcoded_t_apog,fit_V,fit_T,*optA)
print("normal fit for velocity for t_apog {}".format(shared_t_apog))
print("Got fit with R2 {}".format(np.round(R2_A,5)))
opt0 = a,b,shared_t_apog
t_apog = shared_t_apog


integratedVel = vel_fit(t, *opt0)
x0 = (alt[index1] - altitude_model(t[index1], a, b, t_apog, 0))#*1.04
print("Got x0 {}".format(x0))
maxAlt = altitude_model(t_apog, a, b, t_apog, x0)
maxAltReal = np.max(alt)
print("Got max altitude {}".format(maxAltReal))
print("Predicted max altitude {}".format(maxAlt))
print("Error {}".format(np.abs(maxAltReal-maxAlt)))

print("Got vel at end of timeseries {}".format(integratedVel[-1]))



fig,ax = plt.subplots()
plt.tight_layout()

ax.vlines(t[index0],0,10000,linestyle='dotted',color='k')
ax.vlines(t[index1],0,10000,linestyle='dotted',color='k')

ax.scatter(fit_T, fit_V,marker='^',color='b')


ax.plot(t,alt,color='purple',label='Altitude (m)')
ax.plot(t,vel,color='blue',label='Velocity (m/s)')
ax.plot(t,velMag,color='steelblue',linewidth=3,alpha=0.4,label='Velocity Magnitude (m/s)')


ax.plot(t,integratedVel,label="Velocity Fit",color='blue',linewidth=5,alpha=0.2,zorder=-1)
ax.plot(t, altitude_model(t, a, b, t_apog, x0), label="Altitude Fit", color='purple', linewidth=5, alpha=0.2)


ax.set_xlim(0,t[-1])
ax.set_ylim(0,alt[-1]*1.05)
ax2 = ax.twinx()
ax2.scatter(fit_T, fit_A,marker='^',color='r')
ax2.plot(t[np.argmin(accelZ):],accelZ[np.argmin(accelZ):],color='red',label='Z Acceleration ($\\rm m/s^2$)')
ax2.plot(t[np.argmin(accelZ):], accel_model(t[np.argmin(accelZ):], *opt0), label="Fitted Z Acceleration", color='red', linewidth=5, alpha=0.2)
ax2.plot(t[np.argmin(accelZ):], accel_quintic(t[np.argmin(accelZ):], *opt_A_best), label="Fitted Z Acceleration", color='orange', linewidth=5, alpha=0.5)


ax.plot([-1],[-1],color='red',label='Z Acceleration ($\\rm m/s^2$)')
ax.plot([-1],[-1],color='red',label='Fitted Z Acceleration',linewidth=5, alpha=0.2)
ax.legend(loc='upper left',ncol=1)
ax2.set_ylim(-50,75)
ax2.hlines(-g,*ax.get_xlim(),linestyle='dotted',linewidth=1,color='r')

plt.show()