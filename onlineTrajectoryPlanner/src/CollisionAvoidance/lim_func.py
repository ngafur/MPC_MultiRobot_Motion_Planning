from casadi import exp


def lim_func(x): 
    cc = 10 
    f = x/(1+exp(-cc*x))-(x-1)/(1+exp(-cc*(x-1))) 
    return f