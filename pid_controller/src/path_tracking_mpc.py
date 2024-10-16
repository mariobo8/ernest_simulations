from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import math
import matplotlib.pyplot as plt

from simulation_code import simulate

# Weighing Matrices
#states
Q_x = 8e6
Q_y = 2e6
Q_psi = 8e5
Q_theta = 10
#controls
R_v = 10
R_delta = 1
R_virt_v = 1

#reate cannge input
W_v = 1e8
W_delta = 1e9
W_v_virt = 1

#penalty
eps = 1500

# MPC settings
dt = 0.1  # time between steps in seconds
N = 40          # number of look ahead steps
lambd = -0.005

# robot data
rob_diam = 0.3      # diameter of the robot
wheel_radius = 1    # wheel radius
Lx = 0.4            # L in J Matrix (half robot x-axis length)
Ly = 0.4            # l in J Matrix (half robot y-axis length)
sim_time = 40      # simulation time

#initial path
theta_0 = -30
theta_f = 0

# Initial position
x_init = -30
y_init = 3
psi_init = 0
theta_init = theta_0

# Target Position
x_target = 15
y_target = 10
psi_target = pi/4

# Boundaries
v_max = 1
delta_max = 0.60
virtual_v_max = 1
v_min = 0
delta_min = -0.60
virtual_v_min = 0

# Shift function
def shift_timestep(dt, t0, x0, u, f, f2, theta_0, theta_prev):
    st = x0
    con = u[:, 0]
    #print("x0", x0)
    f_value = f(st, con)
    x0 = ca.DM.full(st + (dt * f_value))
    
    u = u.T
    u0 = ca.vertcat(
        u[1:, :],
        u[-1,:]
    )
    theta = x0[3]
    rho = ca.DM.full(f2(theta))
    rho_prev = ca.DM.full(f2(theta_0))
    rho_prev_prev = ca.DM.full(f2(theta_prev))
    drho_dtheta = (rho - rho_prev)/ \
                  (theta - theta_0)


    d2rho_dtheta2 = (rho - 2 * (rho - rho_prev) + (rho - rho_prev_prev))/ \
                    ((theta - theta_0)*(theta_0-theta_prev))
    

    
    up0_1 = (theta-theta_0)/dt * np.sqrt(1+(drho_dtheta)**2)
    up0_2 = np.arctan((1+(drho_dtheta**2)**(-3/2))*d2rho_dtheta2)
    up0 = ca.DM([[up0_1.__float__()], 
                [up0_2.__float__()],
                [0.0]])
    up0 = DM2Arr(up0)
    
    xp0 = ca.DM([[theta.__float__()],
                [rho.__float__()], 
                [np.arctan(drho_dtheta).__float__()], 
                [0.0]])
    xp0 = DM2Arr(xp0)
    #print("xp0", xp0)
    t0 = t0 + dt

    return t0, x0, u0, xp0, up0

# change from  matrix to array  
def DM2Arr(dm):
    return np.array(dm.full())


# Define state variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
psi = ca.SX.sym('psi')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    psi,
    theta
)
n_states = states.numel()

# Define control variables
v = ca.SX.sym('v')
delta = ca.SX.sym('delta')
virtual_v = ca.SX.sym('virtual_v')
controls = ca.vertcat(
    v,
    delta,
    virtual_v
)
n_controls = controls.numel()

#define the path
#mu = [6, 20, 5, 0.35];  
#rho = - mu[0] * np.log(mu[1] / (mu[2] - theta)) * sin(mu[3] * theta)
rho = theta/6 + 5
## build matrices containg states and controls

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N)

# coloumn vector for storing initial state and target state
P = ca.SX.sym('P', n_states + n_states + n_controls)

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_x, Q_y, Q_psi, Q_theta)

# controls weights matrix
R = ca.diagcat(R_v, R_delta, R_virt_v)

#rate changing matrix
W = ca.diagcat(W_v, W_delta, W_v_virt)

# righthand-side function
RHS = ca.vertcat(v*cos(psi),
                 v*sin(psi),
                 v*np.tan(delta),
                -lambd*(-30)+virtual_v)

# maps controls from [input] to [states].T
f = ca.Function('f', [states, controls], [RHS])


obj = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation


# Multiple shooting
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    if (k == N-1):  
        con_l = con
    else:
        con_l = U[:,k+1]
        
    obj = obj \
        + (st - P[4:8]).T @ Q @ (st - P[4:8]) \
        + (con-P[8:11]).T @ R @ (con-P[8:11]) \
        + (con - con_l).T @ W @ (con - con_l) + eps/2*st[3]**2
    
    st_next = X[:, k+1]
    f_value = f(st, con)
    st_next_euler = st + dt * f_value
    g = ca.vertcat(g, st_next - st_next_euler)


OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)

nlp_prob = {
    'f': obj,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = -30         # X lower bound
lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # psi lower bound
lbx[3: n_states*(N+1): n_states] = -30         # theta lower bound


ubx[0: n_states*(N+1): n_states] = 0           # X upper bound
ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound
ubx[3: n_states*(N+1): n_states] = 0           # theta lower bound

#input lower bound
lbx[n_states*(N+1):n_states*(N+1)+n_controls*N:n_controls] = v_min                
lbx[n_states*(N+1)+1:n_states*(N+1)+n_controls*N:n_controls] = delta_min
lbx[n_states*(N+1)+2:n_states*(N+1)+n_controls*N:n_controls] = virtual_v_min

#input upper bound
ubx[n_states*(N+1):n_states*(N+1)+n_controls*N-n_controls:n_controls] = v_max                                                                            # v upper bound for all V
ubx[n_states*(N+1)+1:n_states*(N+1)+n_controls*N-n_controls+1:n_controls] = delta_max   
ubx[n_states*(N+1)+2:n_states*(N+1)+n_controls*N-n_controls+2:n_controls] = virtual_v_max 

args = {
    'lbg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
    'ubg': ca.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0

# function to compute values of the path

f2 = ca.Function('f2', [theta],[rho])
rho_0 = ca.DM.full(f2(theta_0))
rho_f = ca.DM.full(f2(theta_f))
#initial conditions
x0 = ca.DM([x_init, y_init, psi_init, theta_init])        # initial state
xp0 = ca.DM([theta_0, rho_0, -1, 0])
up0 =ca.DM([0, 0, 0])

state_target = ca.DM([theta_f, rho_f, 0, 0])  # target state


# xx = DM(x0)
t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))  # initial control
X0 = ca.repmat(x0, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])
theta_0 = -30

###############################################################################

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    #while (mpc_iter * dt < sim_time):
    while (ca.norm_2(x0 - state_target) > 1e-2) and (mpc_iter * dt < sim_time):
    #while (mpc_iter<2):
        t1 = time()
        args['p'] = ca.vertcat(
            x0,    # current state
            xp0,   # target state
            up0
        )

        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )
        print(type(args['ubg']))
        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        theta_prev = theta_0
        theta_0 = x0[3]
        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))
        
        t0, x0, u0, xp0, up0 = shift_timestep(dt, t0, x0, u, f, f2, theta_0, theta_prev)
        
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        #print(mpc_iter)
        #print(t2-t1)
        times = np.vstack((
            times,
            t2-t1
        ))

        mpc_iter = mpc_iter + 1

    main_loop_time = time()
    ss_error = ca.norm_2(x0 - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    #simulate(cat_states, cat_controls, times, dt, N, save=False)

