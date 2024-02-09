import casadi as ca
import numpy as np 

# Shift function
def shift_timestep(dt, t0, x0, u, f, f2, theta_0, theta_prev):
    
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
    print("xp0", xp0)
    t0 = t0 + dt

    return t0, x0, u0, xp0, up0

# change from  matrix to array  
def DM2Arr(dm):
    return np.array(dm.full())

def Arr2DM(dm):
    return ca.DM(dm)
