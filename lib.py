import numpy as np
import matplotlib.pyplot as plt
import gekko
from gekko import GEKKO
import json
from scipy import optimize


def run_optimal_control(
    x_0,            # initial state
    x_ref,          # final reference state  
    N,              # length of the time horizon
    time_step,      # time step
    f,              # differential equation
    h,              # path inequality constraints
#     h_f,            # final inequality constraint
    F,              # objective function
    integer=False,  # integrality flag
    const=False,    # fixator
    const_i=None,   # fixed i value
    name=''         # name of the problem
):
    """
    Function for MINL Optimal Control solving
    """
    m = GEKKO(remote=False)
    m.time = np.arange(N + 1) * time_step

    m.options.SOLVER = 1 if integer else 3

    if integer:
        m.solver_options = [
            'minlp_maximum_iterations 5000', # 10.000
            'minlp_max_iter_with_int_sol 500', # 500
            'minlp_as_nlp 0', # 1
            'minlp_branch_method 2', # 3
            'minlp_gap_tol 0.0005', # 0.01
            'minlp_integer_tol 0.005', # 0.01
        ]
        
    # Manipulated variable: continuous
    u = m.Var(value=0.0, lb=0, ub=1, name='u')
    
    # Manipulated variable: integer
    i = m.Var(value=0.0, integer=integer, lb=0, ub=1, name='i') 
    if const:
        for n in range(N+1):
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
    history['obj_value'] = sum(history['F'])
    
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
    F, F_L,
    name=''         # name of the problem
):
    """
    Function for MI Quadratic Optimal Control solving
    """
    m = GEKKO(remote=False)
    m.time = np.arange(N + 1) * time_step

    m.options.SOLVER = 1

    m.solver_options = ['minlp_maximum_iterations 1000',
                        'minlp_max_iter_with_int_sol 500',
                        'minlp_as_nlp 0',
                        'minlp_branch_method 2',
                        'minlp_integer_tol 0.001',
                        'minlp_gap_tol 0.001',
                        'nlp_maximum_iterations 50']
        
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
    
    for n in range(N+1):
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
#     m.Obj(F(x, u, i))
    
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
    history['F_L']  = [F_L(x, x0) for x, x0 in xx0_list]
    history['F_GN'] = [F_GN(x, x0) for x, x0 in xx0_list]
    history['f'] = results['f_var']
    history['h'] = results['h_var']
    
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_value'] = sum(history['F'])
    
    return history

def linear(f, x, x0):
    """
    Linearization of the function f
    """
    f_ = lambda x: f(*x)
    if type(x0[0]) == gekko.gk_variable.GKVariable:
        x0 = [float(x0_i.value.__array__()) for x0_i in x0]
    f_grad = optimize.approx_fprime(x0, f_, 1e-6)
    delta = np.array(x) - np.array(x0)
    return f_(x0) + f_grad @ delta

def quadratic(f, x, x0, B):
    """
    Quadratization of the function f
    """
    f_linear = linear(f, x, x0)
    delta = np.array(x) - np.array(x0)
    return f_linear + 0.5 * delta.T @ B @ delta

def b_gn(f, x, x0):
    """
    Gaussian-Newton Hessian approximation 
    """
    f_ = lambda x: f(*x)
    if type(x0[0]) == gekko.gk_variable.GKVariable:
        x0 = [float(x0_i.value.__array__()) for x0_i in x0]
    f_grad = optimize.approx_fprime(x0, f_, 1e-6)
    if f_grad.ndim == 2:
        return f_grad.T @ f_grad
    return np.outer(f_grad, f_grad)

def pos_semidef(B):
    """
    Positive semidefinition checking 
    """
    return np.all(np.linalg.eigvals(B) >= 0)

def gauss_newton(f, f1, x, x0):
    """
    Gauss-Newton method
    """
    B_GN = b_gn(f1, x, x0)
#     print(f'Pos.semidef.: {pos_semidef(B_GN)}')
    return quadratic(f, x, x0, B_GN)