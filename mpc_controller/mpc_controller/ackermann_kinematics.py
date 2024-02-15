from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from casadi import sin, cos, pi
from .utils import *
from rclpy.node import Node

import time 

import casadi as ca
import numpy as np


class AckermannKinematics(object):
    def __init__(self):
        self.N = 40
        self.theta = -5
        self.x0 = ca.DM([-5, 0, 1.57, -5])      
        self.X0 = ca.repmat(self.x0, 1, self.N+1)   
        self.u0 = ca.DM.zeros((3, self.N))
        self.xp0 = ca.DM([-5, 0, 0, 0])
        self.up0 =ca.DM([0, 0, 0])
        self.theta_0 = -5
        

    def dimensions(self):
        L = 0.8
        wheel_radius = 0.15
        return L, wheel_radius
    
    
    def problem_settings(self):
        N = 40
        dt = 0.1
        return N, dt
    
    # Shift function
    def shift_timestep( u, f2, theta_prev, X0):
        dt = 0.1
        u = u.T
        u0 = ca.vertcat(
            u[1:, :],
            u[-1,:]
        )
        #print("pred_state")
        #print(X0[:,1])
        theta_0 = X0[3,0] 
        theta = X0[3,1]
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


        return  u0, xp0, up0, theta, theta_0
    
    def kin_model(self):
        [L, wheel_radius] = self.dimensions()
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

        # righthand-side function
        RHS = ca.vertcat(v*cos(psi),
                        v*sin(psi),
                        v*np.tan(delta)/L,
                        -0.6+virtual_v)

        # maps controls from [input] to [states].T
        f = ca.Function('f', [states, controls], [RHS])
        return states, controls, n_states, n_controls ,f, theta
    
    def path(self, theta):
        #define the path
        rho = theta + 5
        #rho = - mu[0] * np.log(mu[1] / (mu[2] - theta)) * sin(mu[3] * theta)
        f2 = ca.Function('f2', [theta],[rho])
        return f2
    
    def weighing_matrices(self, n_states, n_controls, N):
    # Weighing Matrices
        #states
        Q_x = 8e7
        Q_y = 2e7
        Q_psi = 8e7
        Q_theta = 10

        #controls
        R_v = 10e1
        R_delta = 1e1
        R_virt_v = 1

        #reate cannge input
        W_v = 1e8
        W_delta = 1e9
        W_v_virt = 1

        #penalty
        eps = 1500
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

        return X,U,P,Q,R,W,eps

    def cost_function(self):
        [N, dt] = self.problem_settings()
        [_, _, n_states, n_controls ,f,theta] = self.kin_model()
        f2 = self.path(theta)
        [X,U,P,Q,R,W, eps] = self.weighing_matrices(n_states,n_controls, N)
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
                + (con - P[8:11]).T @ R @ (con - P[8:11]) \
                + (con - con_l).T @ W @ (con - con_l) + eps/2*st[3]**2
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + dt * f_value
            g = ca.vertcat(g, st_next - st_next_euler)
        
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )
        print("Computed obj")
        return obj,OPT_variables, g, P, n_states, n_controls, N, f, f2  
 
    def constraints(self, n_states, n_controls, N):
        # Boundaries
        v_max = 1
        delta_max = 0.60
        virtual_v_max = 1
        v_min = 0
        delta_min = -0.60
        virtual_v_min = 0
        lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

        lbx[0: n_states*(N+1): n_states] = -5         # X lower bound
        lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
        lbx[2: n_states*(N+1): n_states] = -ca.inf     # psi lower bound
        lbx[3: n_states*(N+1): n_states] = -5         # theta lower bound


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
        lbg = ca.DM.zeros((n_states*(N+1), 1))  # constraints lower bound
        ubg = ca.DM.zeros((n_states*(N+1), 1))  # constraints upper bound
        return lbg,ubg,lbx,ubx


    def set_solver(self):
        [obj,OPT_variables, g, P, n_states, n_controls, N, f, f2] = self.cost_function()
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
        [lbg,ubg,lbx,ubx] = self.constraints(n_states, n_controls, N)
        args = {
            'lbg': lbg,
            'ubg': ubg,
            'lbx': lbx,
            'ubx': ubx
        }

        return solver, args, n_states, n_controls, f, f2
    
    def solve_mpc(self, solver, state, args, n_states, n_controls, xp0, up0, f2):
        state[3] = self.theta
        args['p'] = ca.vertcat(
            state,    # current state
            xp0,   # target state
            up0
            )


        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(self.X0, n_states*(self.N+1), 1),
            ca.reshape(self.u0, n_controls*self.N, 1)
        )

        #print(type(args['lbx']))

        #print("Argument types and values:") 
        #for key, value in args.items():
        #    print(f"{key}: {type(value)}, {value}")
        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        u = ca.reshape(sol['x'][n_states * (self.N + 1):], n_controls, self.N)
        self.X0 = ca.reshape(sol['x'][: n_states * (self.N+1)], n_states, self.N+1)
        #print(self.X0)
        theta_prev = self.theta_0

        #t0, x0, u0, xp0, up0 = shift_timestep(0, 0, state, u, f, f2, theta_0, theta_prev)
        inp = DM2Arr(u[:, 0])
        input = [inp[0], 0.0, inp[1], 0.0, 0.0]
        #input = np.array([1.0 ,0.5])
        #print(state)
        [self.u0, new_xp0, new_up0, self.theta, self.theta_0] = self.shift_timestep(u, f2, theta_prev, self.X0)
        #print("theta-2 theta-1 theta")
        #print("reference position")
        #print(self.xp0)
        #print(theta_prev, self.theta_0, self.theta)
        return input, new_xp0, new_up0


