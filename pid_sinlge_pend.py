import scipy as sp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import KDTree
from scipy.interpolate import LinearNDInterpolator

#This is sum bullshit cuz the numpy inter function was giving sus outputs
def inter(x, xp, fp):
    x0, x1 = xp
    f0, f1 = fp
    return (x-x0)*(f1-f0)/(x1-x0) + f0
5


# Bob mass (kg), pendulum length (m), acceleration due to gravity (m.s-2).
m, len, grav = 1, 0.2, 9.81
# Initial angular displacement (rad), tangential velocity (m.s-1)
theta0, theta_dot0 = np.radians(250), 0

duration = 120 #seconds
fps = 45
target = np.radians(180)
P = 15
I = 0
D = 0

def controller(target, value, args, params = (P,I,D)):
    delta_t, last_p, i = args
    kp, ki, kd = params

    #print(delta_t)
    p = value-target
    i += p*delta_t
    d = (p-last_p) / delta_t

    sys_input = kp*p + ki*i + kd*d

    return p, i, d, sys_input

err = 0
#state space derivative
def pendulum(t, params, l=len, g=grav):
    last_t, theta, theta_dot, p, i, v, x = params

    #err += 3*(np.random.rand()-0.5)

    p, i, d, a = controller(target, theta, (t-last_t, p, i))



    theta_ddot = -0.1*theta_dot - g*np.sin(theta)  + a*np.cos(theta)

    #derivative of the state space
    return 1, theta_dot, theta_ddot, d, p, a, v

y_init=[0, theta0, theta_dot0, 0, 0, 0, 0]
#solution to the pendulum ivp over the specified time range 
sol = sp.integrate.solve_ivp(fun=pendulum, t_span=[0.0, duration], y0=y_init, max_step=0.01, min_step=0.001)


#KDTree and array are used as lookup table
time_tree = KDTree([[t,] for t in sol.t])
states = sol.y


# Initialize the animation plot. Make the aspect ratio equal so it looks right.
fig = plt.figure()
ax = fig.add_subplot(aspect='equal')

# Set the plot limits so that the pendulum has room to swing!
ax.set_xlim(-len*1.2, len*1.2)
ax.set_ylim(-len*1.2, len*1.2)


def get_r(state):
    last_t, th, th_dot, p, i, v, x = state
    """Return the (x, y) coordinates of the bob from the state variables"""
    return len * np.sin(th), -len * np.cos(th)


# The pendulum rod, in its initial position.
x0, y0 = get_r(y_init)
line, = ax.plot([0, x0], [0, y0], lw=3, c='k')
# The pendulum bob: set zorder so that it is drawn over the pendulum rod.
circle = ax.add_patch(plt.Circle([x0, y0], len/8, fc='r', zorder=3))

def animate(i, args):
    """Update the animation at frame i."""    
    times = args
    dists, inds = time_tree.query(times[i], k=2)

    nearest_times = [time_tree.data[j,0] for j in inds]
    sys_state = [inter(times[i], nearest_times, [state_var[j] for j in inds]) for state_var in states]

    x, y = get_r(sys_state)
    line.set_data([0, x], [0, y])
    circle.set_center((x, y))

#Sets up and runs animation
nframes = duration*fps
times = np.linspace(0,duration, num=nframes) 
ani = animation.FuncAnimation(fig, animate, frames=nframes, repeat=False, interval=1000/fps, fargs = (times,))
plt.show()


#plt.plot(time_tree.data, states[1], 'r')
plt.plot(time_tree.data, states[3], 'b')#theta position
plt.plot(time_tree.data, states[6], 'g')#x position

plt.show()

