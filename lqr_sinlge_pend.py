import scipy as sp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import KDTree
from scipy.interpolate import LinearNDInterpolator
from scipy.optimize import fsolve
import control

#This is sum bullshit cuz the numpy inter function was giving sus outputs
def inter(x, xp, fp):
    x0, x1 = xp
    f0, f1 = fp
    return (x-x0)*(f1-f0)/(x1-x0) + f0



# Bob mass (kg), pendulum length (m), acceleration due to gravity (m.s-2).
mass, len, grav, c = 0.1, 0.3, 9.81, 0.01

# Initial angular displacement (rad), tangential velocity (m.s-1)
th0, th_dot0, x0, x_dot0 = np.radians(180), 0, 0, 0

#animation params
duration = 30 #seconds
fps = 45

fp_up = np.array([np.radians(180), 0, 0, 0])

#state err cost
q = np.diag([100, 30, 500, 10])

#actuation cost
r = np.array([[0.04]])

#jacobian of state space function
def jac(y, g=grav, l=len, m=mass):
    th, th_dot, x, x_dot = y
    return (np.array([[0, 1, 0, 0],
                     [-(g*np.cos(th))/l, -c/(m*(l**2)), 0, 0],
                     [0, 0, 0, 1],
                     [0, 0, 0, 0]]),

             np.array([[0, -np.cos(th)/l, 0, 1]]).T)

#fixed point linearized matrix around up pos
a_up , b_up = jac(fp_up)

print(np.linalg.matrix_rank(control.ctrb(a_up, b_up)))
p = sp.linalg.solve_continuous_are(a_up, b_up, q, r, s=fp_up[:, None])
#print(p)
k = np.linalg.inv(r).dot(b_up.T).dot(p)
#k = sp.signal.place_poles(a_up,b_up,[-10.1,-10.2,-10.3,-10.4]).gain_matrix
#print(a_up - b_up*k)
print(np.linalg.eigvals(a_up - b_up*k))


#state space derivative
def pendulum(t, y, l=len, g=grav, m=mass, K=k):
    th, th_dot, x, x_dot = y

    #this alternates between x positions every 5 seconds
    if int(t/5)%2 == 0:
        x_target = 0.3
    else: 
        x_target = -0.3

    u = -np.dot(K, (y-fp_up - [0,0,x_target,0]))
    #print(u)
    x_ddot = u[0]

    th_ddot = -(x_ddot*np.cos(th) + g*np.sin(th))/l - c*th_dot/(m*(l**2))

    return np.array([th_dot, th_ddot, x_dot, x_ddot])


#solution to the pendulum ivp over the specified time range 
y_init = np.array([th0, th_dot0, x0, x_dot0])
sol = sp.integrate.solve_ivp(fun=pendulum, t_span=[0.0, duration], y0=y_init, max_step=0.01)


#KDTree and array are used as lookup table for interpolation of results
time_tree = KDTree([[t,] for t in sol.t])
states = sol.y

# Initialize the animation plot. Make the aspect ratio equal so it looks right.
fig = plt.figure()
ax = fig.add_subplot(aspect='equal')

# Set the plot limits so that the pendulum has room to swing
w=3
h=1.3
ax.set_xlim(-len*w, len*w)
ax.set_ylim(-len*h, len*h)

def get_r(y):
    th, th_dot, x, x_dot= y
    """Return the (x, y) coordinates of the bob from the state variables"""
    return x, len*np.sin(th) + x, -len*np.cos(th)

# The pendulum rod, in its initial position.
x, bob_x0, bob_y0 = get_r(y_init)
line, = ax.plot([x, bob_x0], [0, bob_y0], lw=3, c='k')
# The pendulum bob: set zorder so that it is drawn over the pendulum rod.
circle = ax.add_patch(plt.Circle([bob_x0, bob_y0], len/8, fc='r', zorder=3))

def animate(i, args):
    """Update the animation at frame i"""    
    times = args
    dists, inds = time_tree.query(times[i], k=2)

    nearest_times = [time_tree.data[j,0] for j in inds]
    sys_state = [inter(times[i], nearest_times, [state_var[j] for j in inds]) for state_var in states]

    x, bob_x, bob_y = get_r(sys_state)
    line.set_data([x, bob_x], [0, bob_y])
    circle.set_center((bob_x, bob_y))

#Sets up and runs animation
nframes = duration*fps
times = np.linspace(0,duration, num=nframes) 
ani = animation.FuncAnimation(fig, animate, frames=nframes, repeat=False, interval=1000/fps, fargs = (times,))
plt.show()

plt.plot(time_tree.data, states[0], 'b')#theta position
plt.plot(time_tree.data, states[2], 'r')#x position

plt.show()

