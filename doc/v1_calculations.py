import numpy as np
import scipy.optimize as spopt
import scipy.special as spspec
import matplotlib.pyplot as plt

# Create the plot object
m = 53.48
rho = 0.736
airbrakesCd = 1.28
rocketCd = 0.5859
aRef = 0.01929
k = 11.12
v_0 = 351.0
x_d = 5000.0
x_0 = (v_0**2)/(2*k) - 5358.82
g = 9.81
B = (aRef * rho * rocketCd) / (2*m)

Dx = 100
alpha = 0.2463


rightSideOfExpression = (2*(B**(3/2))*m*g*Dx)/(airbrakesCd*rho*(alpha**(3/2)))
print(rightSideOfExpression)

# 1.875 for 300m
# 1.249 for 200m
# 0.625 for 100m

exit(0)






fig,ax = plt.subplots(figsize=(5,5))

# Cosmetic adjustments:
plt.subplots_adjust(bottom=0.12,
	top=0.97,left=0.12,right=0.96)



def A_of_t(t): 
    
    deltaX = x_d - v_0*v_0/2/k + x_0
    deltaK = -2*k*k/v_0/v_0*deltaX

    a = (2*m)/(rho*airbrakesCd)
    b = (-2*(k**2)*((x_d-(v_0**2))/(2*k + x_0))) + (v_0**2)*(k-g)
    c = b/((v_0**2)*(((k)*t-v_0)**2))
    d = -4*t*(k**2)*((x_d-(v_0**2))/(2*k + x_0))
    e = d/((v_0**2)*((k)*t-v_0))
    f = B * (1 + e)
    toReturn =  a * (c-f)
    
    return np.minimum(toReturn, 10.0)




# Plotting:
t = np.linspace(17.5, 31.55, 1000)
A = A_of_t(t)
v = v_0 - k*t
a = -k 
x = x_0 + v_0*t - 0.5*k*(t**2)


print(-m*g-(B*m*v*v)-(m*a))
exit(0)

ax.plot(t, x, color='blue', linewidth=2, label='x')
ax.plot(t, v, color='red', linewidth=2, label='v')
ax2 = ax.twinx()
ax2.plot(t, A, color='green', linewidth=2, label='A')
ax.hlines(0, xmin=t[0], xmax=t[-1], colors='black', linestyles='dashed', linewidth=1)
ax.legend()
ax2.legend()

# Saving:
plt.savefig("graph_A_of_t.pdf")
plt.show()
