import numpy as np
import matplotlib.pyplot as plt
import gekko
from gekko import GEKKO
import json
from .math import *
import os


def run_optimal_control(
    x_0,            # initial state
    x_ref,          # final reference state  
    N,              # length of the time horizon
    time_step,      # time step
    f,              # differential equation
    h,              # path inequality constraints
    F,              # objective function
    options=None,   # gekko solver options
    integer=False,  # integrality flag
    const=False,    # fixator
    const_i=None,   # fixed i value
    name='',        # name of the problem
    disp=False
):
    """
    Function for MINL Optimal Control solving
    """
    m = GEKKO(remote=False)
    m.time = np.arange(N + 1) * time_step

    m.options.SOLVER = 1 if integer else 3
    if integer and options is not None:
        m.solver_options = options
        
    # Manipulated variable: continuous
    u = m.Var(value=0.0, lb=0, ub=1, name='u')
    
    # Manipulated variable: integer
    i = m.Var(value=0.0, integer=integer, lb=0, ub=1, name='i') 
    if const:
        for n in range(1, N+1):
            m.fix(i, const_i[n], pos=n)

    # Controlled Variable
    x = m.Var(value=x_0, name='x')

    # Equations
    f_var = m.Var(value=0.0, name='f_var')
    h_var = m.Var(value=0.0, name='h_var')
    m.Equation(f_var == x.dt() - f(x, u, i)) # f function
    m.Equation(h_var == h(x, u, i))          # h function
    m.Equation(f_var == 0)
    m.Equation(h_var <= 0)
    
    # P polygon
    a = m.Var()
    b = m.Var()
    c = m.Var()
    m.delay(i, a, 1)
    m.delay(i, b, 2)
    m.delay(i, c, 3)
    
    m.Equation(a - b - i <= 0)
    m.Equation(a - c - i <= 0)
    
    # Objective function
    m.Obj(F(x, u, i)) # F function

    m.options.IMODE = 6
    m.solve(disp=disp)
    # -----------------------------------------------------
    with open(m.path+'//results.json') as file:
        results = json.load(file)
        
    history = {}
    history['name_of_the_problem'] = name
    history['time'] = results['time']
    
    history['x'] = x = results['x']
    history['u'] = u = results['u']
    history['i'] = i = results['int_i' if integer else 'i']
    
    x_list = list(zip(x, u, i))
    history['F'] = [F(*x) for x in x_list]
    history['f'] = results['f_var']
    history['h'] = results['h_var']
    
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_value'] = m.options.OBJFCNVAL

    return history

def run_optimal_control_dist(
    x_0,            # initial state
    x_ref,          # final reference state 
    x_nlp,          # x* NLP solution
    u_nlp,          # u* NLP solution
    i_nlp,          # i* NLP solution
    N,              # length of the time horizon
    time_step,      # time step
    f_L,            # linearization of f
    h_L,            # linearization of h
    F_GN,           # Gauss-Newton of F
    options,        # gekko solver options
    F,              # original F for result comparing
    name='',        # name of the problem
    const=False,    # fixator
    const_i=None,   # fixed i value
):
    """
    Function for MI Quadratic Optimal Control solving
    """
    m = GEKKO(remote=False)
    m.time = np.arange(N + 1) * time_step

    m.options.SOLVER = 1
    if options is not None:
        m.solver_options = options
        
    # Manipulated variable: continuous
    u = m.Var(value=0.0, lb=0, ub=1, name='u')

    # Manipulated variable: integer
    i = m.Var(value=0.0, integer=True, lb=0, ub=1, name='i')
    if const:
        for n in range(1, N+1):
            m.fix(i, const_i[n], pos=n)
            
    # Controlled Variable
    x = m.Var(value=x_0, name='x')

    # Fixed points for approximation
    u0 = m.Var(value=0.0, name='u0')
    i0 = m.Var(value=0.0, name='i0')
    x0 = m.Var(value=x_0, name='x0')
    
    for n in range(1, N+1):
        m.fix(u0, u_nlp[n], pos=n)
        m.fix(i0, i_nlp[n], pos=n)
        m.fix(x0, x_nlp[n], pos=n)

    # Equations
    f_var = m.Var(value=0.0, name='f_var')
    h_var = m.Var(value=0.0, name='h_var')
    m.Equation(f_var == x.dt() - f_L((x, u, i), (x0, u0, i0))) # f function
    m.Equation(h_var == h_L((x, u, i), (x0, u0, i0)))          # h function
    m.Equation(f_var == 0)
    m.Equation(h_var <= 0)
    
    # P polygon
    a = m.Var()
    b = m.Var()
    c = m.Var()
    m.delay(i, a, 1)
    m.delay(i, b, 2)
    m.delay(i, c, 3)

    m.Equation(a - b - i <= 0)
    m.Equation(a - c - i <= 0)

    # Objective function
    m.Obj(F_GN((x, u, i), (x0, u0, i0)))
    
    m.options.IMODE = 6
    m.solve(disp=False)
    # -----------------------------------------------------
    with open(m.path+'//results.json') as file:
        results = json.load(file)
        
    history = {}
    history['name_of_the_problem'] = name
    history['time'] = results['time']
    
    history['x'] = x = results['x']
    history['u'] = u = results['u']
    history['i'] = i = results['int_i']
    
    history['x0'] = x0 = results['x0']
    history['u0'] = u0 = results['u0']
    history['i0'] = i0 = results['i0']
    
    x_list   = list(zip(x, u, i))
    x0_list  = list(zip(x0, u0, i0))
    xx0_list = list(zip(x_list, x0_list))

    history['F0']   = [F(*x0) for x0 in x0_list]
    history['F']    = [F(*x) for x in x_list]
    history['F_GN'] = [F_GN(x, x0) for x, x0 in xx0_list]
    history['f'] = results['f_var']
    history['h'] = results['h_var']
    
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_value'] = m.options.OBJFCNVAL

    return history
    
def experiment(x_0, x_ref, N, time_step, f, h, F, options):
    """
    """
    history_minlp = run_optimal_control(x_0, x_ref, N, time_step, f, h, F, 
                                        options, integer=True, name='MINLP')
    history_nlp = run_optimal_control(x_0, x_ref, N, time_step, f, h, F, 
                                      options, integer=False, name='NLP')
    x_nlp = history_nlp['x']
    u_nlp = history_nlp['u']
    i_nlp = history_nlp['i']
    obj_nlp = history_nlp['obj_value']

    f_L  = lambda x, x0: linear(f, x, x0)
    h_L  = lambda x, x0: linear(h, x, x0)
    F_L  = lambda x, x0: linear(F, x, x0)
    F_QP = lambda x, x0: gauss_newton(F, F1, x, x0)
    F_GN = lambda x, x0: F_QP(x, x0) - obj_nlp

    history_dist = run_optimal_control_dist(x_0, x_ref, x_nlp, u_nlp, i_nlp, N, 
                                            time_step, f_L, h_L, F_GN, options, F, 'DIST')
    history_miqp = run_optimal_control(x_0, x_ref, N, time_step, f, h, F, options, integer=True, 
                                       const=True, const_i=history_dist['i'], name='MIQP')

    compare_results(history_minlp, history_nlp, history_miqp)