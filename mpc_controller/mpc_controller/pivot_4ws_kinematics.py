from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from casadi import sin, cos, pi
from .utils import *
from rclpy.node import Node

import time 
import os 
import casadi as ca
import numpy as np



class Pivot4wsKinematics(object):
    def __init__(self):
        self.N = 10
        self.dt = 0.1
        self.theta = 0
        [self.x_p, self.y_p, self.arc_length] = self.path(file_name = "std_path.txt")
        self.start = self.x_p[0]
        self.x0 = ca.DM([self.x_p[0], self.y_p[0], 0.0, self.theta])  
        self.X0 = ca.repmat(self.x0, 1, self.N+1)     
        self.xp0 = []
        for jj in range(1, self.N + 1):
            self.xp0.extend([self.x_p[0], self.y_p[0], 0.0, 0.0])  # initial condition path
        self.xp0 = Arr2DM(self.xp0)
        self.u0 = ca.DM.zeros((6, self.N))
        self.up0 =ca.DM.zeros(6,1)
        self.pred = np.zeros((1,4))
        
        self.ref = np.zeros((1,3))
        self.x_r = ([self.x_p[0], self.y_p[0], 0.0, 0.0])
        self.b = 0.4 #[m]
        self.l_f = 0.16
        self.l_r = 0.71
        self.wheel_radius = 0.15

        self.alpha_prev = 0.0


    def make_vel(self, vf, vr, d_f, d_r, alpha, b):
        beta = np.arctan((self.l_f * np.tan(d_r) + self.l_r * np.tan(d_f)*cos(alpha)+
                          sin(alpha)*self.l_r) / (self.l_f + self.l_r * 
                           (cos(alpha - np.tan(d_f) * sin(alpha)))))
        if (d_f + alpha - d_r) == 0.0: 
            v_fl = vf; v_fr = vf
            v_rl = vr; v_rr = vr
            d_fl = d_f; d_fr = d_f
            d_rl = d_r; d_rr = d_r
        else:
            R = (self.l_f + self.l_r) / (np.tan(d_f)*cos(alpha+beta)+sin(alpha-beta)
                                            + sin(beta) - np.tan(d_r)*cos(beta)) 
            R_fr = np.sqrt((R*cos(beta) + (b - self.l_f*np.tan(alpha))*cos(alpha))**2 + 
                        (R*sin(beta) + self.l_f*cos(alpha) + b*sin(alpha))**2)
            R_fl = np.sqrt((R*cos(beta) - (b + self.l_f*np.tan(alpha))*cos(alpha))**2 + 
                        (R*sin(beta) + self.l_f*cos(alpha) - b*sin(alpha))**2)
            R_f = np.sqrt((R*cos(beta) - self.l_f*sin(alpha))**2 + (R*sin(beta + self.l_f*cos(alpha)))**2)
            
            R_rl = np.sqrt((R*cos(beta) - b)**2 + (R*sin(beta) - self.l_r)**2)
            R_rr = np.sqrt((R*cos(beta) + b)**2 + (R*sin(beta) - self.l_r)**2)
            R_r = np.sqrt((R*cos(beta))**2 + (R*sin(beta) - self.l_r)**2)

            v_fl = R_fl / R_f * vf; v_fr = R_fr / R_f * vf
            v_rl = R_rl / R_r * vr; v_rr = R_rr / R_r * vr
            
            if b * sin(d_f) < R_fl:
                d_fl = d_f + np.arcsin(b * sin(d_f) / R_fl)
            else:
                d_fl = d_f
            
            if b * sin(d_f) < R_fr:
                d_fr = d_f - np.arcsin(b * sin(d_f) / R_fr)
            else:
                d_fr = d_f

            if b * sin(d_f) < R_rl:
                d_rl = d_r + np.arcsin(b * sin(d_r) / R_rl)
            else:
                d_rl = d_r

            if b * sin(d_f) < R_rr:
                d_rr = d_r - np.arcsin(b * sin(d_r) / R_rr)
            else:
                d_rr = d_r 
            
            
        

        alpha_dot = (alpha - self.alpha_prev) / self.dt
        
        h = np.sqrt(self.l_f**2 + self.b**2)
        v_fl = v_fl - alpha_dot * h 
        v_fr = v_fr + alpha_dot * h
        self.alpha_prev = alpha

        return v_fl, v_fr, v_rl, v_rr, d_fl, d_fr, d_rl, d_rr
    
    def kin_model(self):
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
        v_f = ca.SX.sym('v_f')
        v_r = ca.SX.sym('v_r')
        delta_f = ca.SX.sym('delta_f')
        delta_r = ca.SX.sym('delta_r')
        alpha = ca.SX.sym('alpha')
        virtual_v = ca.SX.sym('virtual_v')
        controls = ca.vertcat(
            v_f,
            v_r,
            delta_f,
            delta_r,
            alpha,
            virtual_v
        )
        n_controls = controls.numel()
        
        beta = np.arctan((self.l_f * np.tan(delta_r) + self.l_r * (np.tan(delta_f) * cos(alpha) \
                          + sin(alpha))) / (self.l_f + self.l_r * (cos(alpha) - np.tan(delta_f) * sin(alpha))))
        v = (v_f * cos(delta_f + alpha) + v_r * cos(delta_r)) / (2 * cos(beta))
        # righthand-side function
        RHS = ca.vertcat(v*cos(psi + beta), v*sin(psi + beta),
                         v * (np.tan(delta_f) * cos(alpha + beta) + sin(beta) + \
                         sin(alpha - beta) - np.tan(delta_r) * cos(beta)) / (self.l_f + self.l_r),
                         virtual_v)
        ## to change
        # maps controls from [input] to [states].T
        f = ca.Function('f', [states, controls], [RHS])
        return states, controls, n_states, n_controls ,f, s
    
    def path(self, file_name):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        relative_path = "../path"
        file_path = os.path.join(current_dir, relative_path, file_name)
        s_shape_path = np.loadtxt(file_path)
        x_p = s_shape_path[:,0]
        y_p = s_shape_path[:,1]
        arc_length = s_shape_path[:,2]
        #define the path
        return x_p, y_p, arc_length
        
    def weighing_matrices(self, n_states, n_controls, N):
    # Weighing Matrices
        #states
        Q_x = 1e9
        Q_y = 1e9
        Q_psi = 0e0
        Q_s = 0

        #controls
        R_vf = 9e1
        R_vr = 9e1
        R_deltaf = 4e3
        R_deltar = 4e3
        R_alpha = 8e3
        R_virtv = 5e1       

        #rate change input
        
        W_vf = 1e2
        W_vr = 1e2
        W_deltaf = 8e3
        W_deltar = 8e3
        W_alpha = 9e4
        W_virtv = 7e1

        #penalty
        eps = 5e3
        #orientation
        gamma = 2e2
        #anti drifting
        m = 2e7
        
        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', n_states, N + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', n_controls, N)

        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', n_states + n_states + n_controls)

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_psi, Q_s)

        # controls weights matrix
        R = ca.diagcat(R_vf, R_vr, R_deltaf, R_deltar, R_alpha, R_virtv)

        #rate changing matrix
        W = ca.diagcat(W_vf, W_vr, W_deltaf, W_deltar, W_alpha, W_virtv)
        
        return X,U,P,Q,R,W,m,eps,gamma

    def cost_function(self):
        [_, _, n_states, n_controls ,f, _] = self.kin_model()
        [X,U,P,Q,R,W,m,eps,gamma] = self.weighing_matrices(n_states,n_controls, self.N)
        obj = 0  # cost function
        g1 = X[:, 0] - P[:n_states]  # constraints in the equation
        g2 = U[:, 0] - P[-n_controls:]


        # Multiple shooting
        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            vf = U[0,k]; vr = U[1,k]; d_f = U[2,k]; d_r = U[3,k]; a = U[4,k]
            if k == (self.N-1): con_l = con
            else: con_l = U[:,k+1]
            i_state = slice(4, 8)

            obj = obj \
                + (st - P[i_state]).T @ Q @ (st - P[i_state]) \
                + (con).T @ R @ (con) \
                + (con - con_l).T @ W @ (con - con_l) \
                + m*(vf*cos(d_f + a) - vr*cos(d_r))**2 
            
            st_next = X[:, k+1]
            f_value = f(st, con)
            st_next_euler = st + self.dt * f_value
            g1 = ca.vertcat(g1, st_next - st_next_euler)
            if k == (self.N-1): g2 = ca.vertcat(g2, U[:,k] - U[:,k])
            else: g2 = ca.vertcat(g2, U[:,k] - U[:,k+1])
        obj = obj - eps/2*(st[3])**2 + gamma/2*(st[2] - P[6])**2
        

        g = ca.vertcat(g1, g2[:-6])
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )
        print("Computed obj")
        return obj,OPT_variables, g, P, n_states, n_controls,  f
 
    def constraints(self, n_states, n_controls, N):
        # Boundaries
        v_max = 0.5
        alpha_max = 0.4
        delta_max = 0.9
        virtual_v_max = 0.45
        alpha_min = - 0.4
        v_min = -0.3
        delta_min = -0.9
        virtual_v_min = 0
        a_min = - 0.04
        a_max = 0.04
        w_min = - 0.1
        w_max = 0.1
        lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
        
        lbx[0: n_states*(N+1): n_states] = -ca.inf         # X lower bound
        lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
        lbx[2: n_states*(N+1): n_states] = -ca.inf     # psi lower bound
        lbx[3: n_states*(N+1): n_states] = -ca.inf       # theta lower bound

        ubx[0: n_states*(N+1): n_states] = 0           # X upper bound
        ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
        ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound
        ubx[3: n_states*(N+1): n_states] = self.arc_length[-1]           # theta upper bound

        #input lower bound
        lbx[n_states*(N+1):n_states*(N+1)+n_controls*N:n_controls] = v_min                
        lbx[n_states*(N+1)+1:n_states*(N+1)+n_controls*N:n_controls] = v_min
        lbx[n_states*(N+1)+2:n_states*(N+1)+n_controls*N:n_controls] = delta_min
        lbx[n_states*(N+1)+3:n_states*(N+1)+n_controls*N:n_controls] = delta_min                
        lbx[n_states*(N+1)+4:n_states*(N+1)+n_controls*N:n_controls] = alpha_min
        lbx[n_states*(N+1)+5:n_states*(N+1)+n_controls*N:n_controls] = virtual_v_min
        
        #input upper bound
        ubx[n_states*(N+1):n_states*(N+1)+n_controls*N:n_controls] = v_max                                                                            # v upper bound for all V
        ubx[n_states*(N+1)+1:n_states*(N+1)+n_controls*N:n_controls] = v_max   
        ubx[n_states*(N+1)+2:n_states*(N+1)+n_controls*N:n_controls] = delta_max
        ubx[n_states*(N+1)+3:n_states*(N+1)+n_controls*N:n_controls] = delta_max                                                                         # v upper bound for all V
        ubx[n_states*(N+1)+4:n_states*(N+1)+n_controls*N:n_controls] = alpha_max   
        ubx[n_states*(N+1)+5:n_states*(N+1)+n_controls*N:n_controls] = virtual_v_max

        lbg = ca.DM.zeros((n_states*(N+1)+n_controls*N, 1))  # constraints lower bound
        ubg = ca.DM.zeros((n_states*(N+1)+n_controls*N, 1))  # constraints upper bound
    
        #rate input change
        lbg[n_states*(N+1):n_states*(N+1)+n_controls*N:n_controls] = a_min                
        lbg[n_states*(N+1)+1:n_states*(N+1)+n_controls*N:n_controls] = a_min
        lbg[n_states*(N+1)+2:n_states*(N+1)+n_controls*N:n_controls] = w_min*0.30        
        lbg[n_states*(N+1)+3:n_states*(N+1)+n_controls*N:n_controls] = w_min*0.30               
        lbg[n_states*(N+1)+4:n_states*(N+1)+n_controls*N:n_controls] = w_min*0.1
        lbg[n_states*(N+1)+5:n_states*(N+1)+n_controls*N:n_controls] = a_min
        ubg[n_states*(N+1):n_states*(N+1)+n_controls*N:n_controls] = a_max                                                                            # v upper bound for all V
        ubg[n_states*(N+1)+1:n_states*(N+1)+n_controls*N:n_controls] = a_max   
        ubg[n_states*(N+1)+2:n_states*(N+1)+n_controls*N:n_controls] = w_max*0.30        
        ubg[n_states*(N+1)+3:n_states*(N+1)+n_controls*N:n_controls] = w_max*0.30                                                                      # v upper bound for all V
        ubg[n_states*(N+1)+4:n_states*(N+1)+n_controls*N:n_controls] = w_max*0.1
        ubg[n_states*(N+1)+5:n_states*(N+1)+n_controls*N:n_controls] = a_max 
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
                'max_iter': 3000,
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

        index = np.argmin(np.linalg.norm(np.column_stack((x_p, y_p)) \
                                    - np.array([float(X0[0,1]), float(X0[1,1])]), axis=1))
        s_0 = arc_length[index]
        delta_s = (s - s_0) / self.N
        sjj = s_0
        s_prev = s_0
        
        ref = []
        xp0 = []
        for jj in range(1, self.N):
            sjj = s_0 + jj * delta_s
            x_int = np.interp(sjj, arc_length, x_p)
            y_int = np.interp(sjj, arc_length, y_p)
            x_int_prev = np.interp(s_prev, arc_length, x_p)
            y_int_prev = np.interp(s_prev, arc_length, y_p)
            psi_int = np.arctan2((y_int - y_int_prev), (x_int - x_int_prev))
            s_prev = s_0 + max((jj - 1), 0) * delta_s

            xp0.extend([x_int, y_int, psi_int, 0])
        ref.append([float(x_int), float(y_int), float(psi_int)])
        self.ref = np.vstack([self.ref, ref])
        
        xp0.extend([x_int, y_int, psi_int, 0])        
        up0 = u[1,:].T
        xp0 = Arr2DM(xp0)
        x_r = ([x_int, y_int, psi_int, 0])
 
        return  u0, xp0, up0, s, x_r


    def solve_mpc(self, solver, state, args, n_states, n_controls, x_r, up0):
        state[3] = self.theta

        args['p'] = ca.vertcat(
                    state,    # current state
                    x_r,   # target state
                    up0,
                    )
        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(self.X0, n_states*(self.N+1), 1),
            ca.reshape(self.u0, n_controls*self.N, 1)
        )


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

        self.pred = np.vstack([self.pred, self.X0.T])
        inp = DM2Arr(u[:, 0])

            
        v_f = float(inp[0]); v_r = float(inp[1])
        delta_f = float(inp[2]); delta_r = float(inp[3])
        alpha = float(inp[4]); v_virt = float(inp[5])

        [v_fl, v_fr, v_rl, v_rr, 
        delta_fl, delta_fr, delta_rl, delta_rr] = self.make_vel(v_f, v_r, delta_f, delta_r, alpha, self.b)

        input = [v_fl, v_fr, v_rl, v_rr, delta_fl,
                 delta_fr, delta_rl, delta_rr, alpha, v_virt]
        
        [self.u0, new_xp0, new_up0, self.theta, new_x_r] = \
            self.shift_timestep(u, self.X0, self.x_p, self.y_p, self.arc_length, inp)

        return input, new_x_r, new_up0


    