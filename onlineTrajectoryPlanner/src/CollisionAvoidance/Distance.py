from casadi import mtimes
from src.CollisionAvoidance.lim_func import lim_func


def Distance(Rot,H,x0,B_r,r):
    
    alpha1 = -mtimes(mtimes(mtimes(mtimes(((B_r-x0).T),Rot),H),Rot.T),r)/mtimes(mtimes(mtimes(mtimes(r.T,Rot),H),Rot.T),r)
    
    alpha1_c = lim_func(alpha1)
    
    dd = mtimes(mtimes(mtimes(mtimes((B_r+alpha1_c*r-x0).T,Rot),H),Rot.T),(B_r+alpha1_c*r-x0))
    
    return dd