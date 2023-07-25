from src.CollisionAvoidance.lim_func import lim_func
from casadi import mtimes,dot,sqrt


def Line_Distance(B1,r1,B2,r2):
    
    cc = 10.0
    epsilon = 0.001
    alpha1 = 1/(dot(r1,r1)*dot(r2,r2)-dot(r1,r2)*dot(r1,r2)+epsilon) * dot(dot(r2,r2)*r1+dot(r1,r2)*(-r2),(B2-B1))
    alpha1_c = lim_func(alpha1)
    
    alpha2 = 1/(dot(r1,r1)*dot(r2,r2)-dot(r1,r2)*dot(r1,r2)+epsilon) * dot(dot(r1,r2)*r1+dot(r1,r1)*(-r2),(B2-B1))
    alpha2_c = lim_func(alpha2)
    
    dd_q = dot((B1 + alpha1_c*r1 - B2 - alpha2_c*r2),(B1 + alpha1_c*r1 - B2 - alpha2_c*r2))
    dd = sqrt(dd_q)
    
    return dd