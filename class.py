import numpy as np
import matplotlib.pyplot as plt
import gekko
from gekko import GEKKO
import json
from scipy import optimize


class OptimalControl():
    """
    Class for MINL Optimal Control solving
    """
    def __init__(self, x_0, x_ref, N, time_step,
                 f, h, F, solver_options=None,
                 integer_mode=False, 
                 dist_mode=False, dist_kwargs=None,
                 const_mode=False, const_kwargs=None,
                 name=''):
        
        self.time = np.arange(N + 1) * time_step
        self.solver = 1 if integer_mode else 3
        self.solver_options = solver_options
        
        self.f = f
        self.h = h
        self.F = F
        
    # ------------------------------------------
    def init_gekko():
        m = GEKKO(remote=False)
        m.time = self.time

        m.options.SOLVER = self.solver
        if self.solver_options is not None:
            m.solver_options = self.solver_options

        # Manipulated variable: continuous
        u = m.Var(value=0.0, lb=0, ub=1, name='u')

        # Manipulated variable: integer
        i = m.Var(value=0.0, integer=integer, lb=0, ub=1, name='i')

        # Controlled Variable
        x = m.Var(value=x_0, name='x')
        
        return m
    
    
    if const_mode:
        for n in range(1, N+1):
            m.fix(self.i, const_i[n], pos=n)


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
    m.solve(disp=False)
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
    name=''         # name of the problem
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