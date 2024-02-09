import casadi as ca
import numpy as np
from casadi import sin, cos, pi
from .utils import *

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
L = 0.8          # l in J Matrix (half robot y-axis length)
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
mu = [6, 20, 5, 0.35];  
rho = - mu[0] * np.log(mu[1] / (mu[2] - theta)) * sin(mu[3] * theta)


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
                v*np.tan(delta)/L,
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
# nell' inizializzazione del probelma
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


cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])
theta_0 = -30