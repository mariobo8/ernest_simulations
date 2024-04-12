from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from casadi import sin, cos, pi
from ..utils.utils import *
from rclpy.node import Node

import time 
import os 
import casadi as ca
import numpy as np


class AckermannKinematics(object):
    def __init__(self):
        self.N = 10
        self.dt = 0.1
        self.theta = 0
        [self.x_p, self.y_p, self.arc_length] = self.path()
        self.x0 = ca.DM([self.x_p[0], self.y_p[0], 0.0, self.theta])  
        self.X0 = ca.repmat(self.x0, 1, self.N+1)     
        self.xp0 = []
        for jj in range(1, self.N + 1):
            self.xp0.extend([self.x_p[0], self.y_p[0], 0.0, 0.0])  # initial condition path
        self.xp0 = Arr2DM(self.xp0)
        self.u0 = ca.DM.zeros((3, self.N))
        self.up0 =ca.DM.zeros(self.N*3,1)
        

    def dimensions(self):
        l_f = 0.16
        l_r = 0.71
        wheel_radius = 0.161
        return l_f, l_r, wheel_radius
    
    def kin_model(self):
        [l_f, l_r, _] = self.dimensions()
        # Define state variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        s = ca.SX.sym('s')
        states = ca.vertcat(
            x,
            y,
            psi,
            s
        )
        n_states = states.numel()

        # Define control variables
        v_r = ca.SX.sym('v_r')
        delta_f = ca.SX.sym('delta_f')
        virtual_v = ca.SX.sym('virtual_v')
        controls = ca.vertcat(
            v_r,
            delta_f,
            virtual_v
        )
        n_controls = controls.numel()

        beta = np.arctan( l_r * (np.tan(delta_f)) / (l_f + l_r ))
        v = v_r / (2 * cos(beta))
        # righthand-side function
        RHS = ca.vertcat(v*cos(psi + beta), v*sin(psi + beta),
                         v * (np.tan(delta_f) * cos(beta)) 
                               / (l_f + l_r),
                         virtual_v)

        # maps controls from [input] to [states].T
        f = ca.Function('f', [states, controls], [RHS])
        return states, controls, n_states, n_controls ,f, s
    
    def path(self):
        path = os.path.dirname(os.path.realpath(__file__))
        s_shape_path = np.loadtxt(str(path) + '/s_shape_path.txt')
        x_p = s_shape_path[:,0]
        y_p = s_shape_path[:,1]
        arc_length = s_shape_path[:,2]
        #define the path
        return x_p, y_p, arc_length
    
    def weighing_matrices(self, n_states, n_controls, N):
    # Weighing Matrices
        #states
        Q_x = 8e6
        Q_y = 2e6
        Q_psi = 8e3
        Q_theta = 0

        #controls
        R_v = 10e3
        R_delta = 1e4
        R_virt_v = 1

        #reate cannge input
        W_v = 1e7
        W_delta = 1e6
        W_v_virt = 1

        #penalty
        eps = 1e3
        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', n_states, N + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', n_controls, N)

        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', n_states + n_states*N + n_controls*N)

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_psi, Q_theta)

        # controls weights matrix
        R = ca.diagcat(R_v, R_delta, R_virt_v)

        #rate changing matrix
        W = ca.diagcat(W_v, W_delta, W_v_virt)

        return X,U,P,Q,R,W,eps

    def cost_function(self):
        [_, _, n_states, n_controls ,f,_] = self.kin_model()
        [X,U,P,Q,R,W, eps] = self.weighing_matrices(n_states,n_controls, self.N)
        obj = 0  # cost function
        g = X[:, 0] - P[:n_states]  # constraints in the equation


        # Multiple shooting
        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            i_state = slice((k*4+4), (k*4+8))
            i_control =  slice((self.N-1)*4+8+(k*n_controls),\
                              (self.N-1)*4+8+(k*n_controls) + n_controls) #forse da aggiustare
            obj = obj \
                + (st - P[i_state]).T @ Q @ (st - P[i_state]) \
                + (con).T @ R @ (con) \
                + (con - P[i_control]).T @ W @ (con - P[i_control])
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + self.dt * f_value
            g = ca.vertcat(g, st_next - st_next_euler)
        obj = obj - eps/2*(st[3])**2
        
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )
        print("Computed obj")
        return obj,OPT_variables, g, P, n_states, n_controls,  f 
 
    def constraints(self, n_states, n_controls, N):
        # Boundaries
        v_max = 1
        delta_max = 1.05
        virtual_v_max = 1
        v_min = -1
        delta_min = -1.05
        virtual_v_min = 0
        lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

        lbx[0: n_states*(N+1): n_states] = -ca.inf         # X lower bound
        lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
        lbx[2: n_states*(N+1): n_states] = -ca.inf     # psi lower bound
        lbx[3: n_states*(N+1): n_states] = -ca.inf          # theta lower bound


        ubx[0: n_states*(N+1): n_states] = 0           # X upper bound
        ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
        ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound
        ubx[3: n_states*(N+1): n_states] = self.arc_length[-1]       #  theta lower bound

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
        [obj,OPT_variables, g, P, n_states, n_controls, f] = self.cost_function()
        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': g,
            'p': P
        }
        opts = {
            'ipopt': {
                'max_iter': 6000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        [lbg,ubg,lbx,ubx] = self.constraints(n_states, n_controls, self.N)
        args = {
            'lbg': lbg,
            'ubg': ubg,
            'lbx': lbx,
            'ubx': ubx
        }

        return solver, args, n_states, n_controls, f
    
        # Shift function
    def shift_timestep(self, u, X0, x_p, y_p, arc_length, input):
        u = u.T
        u0 = ca.vertcat(
            u[1:, :],
            u[-1,:]
        )
        s = X0[3,1]
        #print("theta")
        #print(s)
        index = np.argmin(np.linalg.norm(np.column_stack((x_p, y_p)) \
                                    - np.array([float(X0[0,0]), float(X0[1,0])]), axis=1))
        #print(float(X0[0,0]))
        s_0 = arc_length[index]
        delta_s = (s - s_0) / self.N
        sjj = s_0
        s_prev = s_0
        
        ref = []
        xp0 = []
        for jj in range(1, self.N):
            sjj = s_0 + jj * delta_s
            #print(s_0, sjj)
            x_int = np.interp(sjj, arc_length, x_p)
            y_int = np.interp(sjj, arc_length, y_p)
            #print(x_int)
            x_int_prev = np.interp(s_prev, arc_length, x_p)
            y_int_prev = np.interp(s_prev, arc_length, y_p)
            psi_int = np.arctan2((y_int - y_int_prev), (x_int - x_int_prev))
            s_prev = s_0 + max((jj - 1), 0) * delta_s

            xp0.extend([x_int, y_int, psi_int, 0])
            ref.append([x_int, y_int, psi_int])
        
        xp0.extend([x_int, y_int, psi_int, 0])
        u0 = np.vstack((u[1:,:], u[-1,:]))
        
        #up0 = np.repeat(u0[:,0], self.N)
        up0 = np.vstack([input] * self.N)
        up0 = Arr2DM(up0)
        xp0 = Arr2DM(xp0)
        #print(xp0)
        #print(input)
        #print("prev input")
        #print(up0)    
        return  u0, xp0, up0, s

    def solve_mpc(self, solver, state, args, n_states, n_controls, xp0, up0):
        state[3] = self.theta
        #print(state)
        #print(ca.DM.size(up0))
        args['p'] = ca.vertcat(
                    state,    # current state
                    xp0,   # target state
                    up0
                    )
        #print(np.size(state))
        #print(np.size(up0))
        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(self.X0, n_states*(self.N+1), 1),
            ca.reshape(self.u0, n_controls*self.N, 1)
        )
        #print(ca.DM.size(args['p']))
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
        #print(self.X0[0,0])
        inp = DM2Arr(u[:, 0])
        #print("state")
        #print(state)

      
        input = [float(inp[0]), float(inp[0]), float(inp[1]), 0.0, 0.0]
       
        [self.u0, new_xp0, new_up0, self.theta] = \
            self.shift_timestep(u, self.X0, self.x_p, self.y_p, self.arc_length, inp)
    
        #print("ref state")
        #print(new_xp0) 
        return input, new_xp0, new_up0


