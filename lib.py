import numpy as np
import matplotlib.pyplot as plt
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

    m.options.SOLVER = 1

    if integer:
        m.solver_options = ['minlp_maximum_iterations 500',
                            'minlp_max_iter_with_int_sol 100',
                            'minlp_as_nlp 0',
                            'nlp_maximum_iterations 50',
                            'minlp_branch_method 1',
                            'minlp_integer_tol 0.05',
                            'minlp_gap_tol 0.01']

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
    m.Equation(f_var == 0)

    m.Equation(h_var == h(x, u, i)) # f function
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
    obj_var = m.Var(value=0.0, name='obj_var')
    m.Equation(obj_var == F(x - x_ref, u, i)) # F function
    m.Obj(obj_var) 

    m.options.IMODE = 6
    m.solve(disp=False)
    
    history = {}
    history['name_of_the_problem'] = name
    
    with open(m.path+'//results.json') as file:
        results = json.load(file)

    history['time'] = results['time']
    
    history['x'] = results['x']
    history['u'] = results['u']
    history['i'] = results['int_i' if integer else 'i']
    
    history['f'] = results['f_var']
    history['h'] = results['h_var']
    history['obj'] = results['obj_var']
    
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_final'] = sum(history['obj']) # m.options.OBJFCNVAL
    
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
    name=''         # name of the problem
):
    """
    Function for MI Quadratic Optimal Control solving
    """
    m = GEKKO(remote=False)
    m.time = np.arange(N + 1) * time_step

    m.options.SOLVER = 1

    m.solver_options = ['minlp_maximum_iterations 500',
                        'minlp_max_iter_with_int_sol 100',
                        'minlp_as_nlp 0',
                        'nlp_maximum_iterations 50',
                        'minlp_branch_method 1',
                        'minlp_integer_tol 0.05',
                        'minlp_gap_tol 0.01']

    # Manipulated variable: continuous
    u = m.Var(value=0.0, lb=0, ub=1, name='u')

    # Manipulated variable: integer
    i = m.Var(value=0.0, integer=True, lb=0, ub=1, name='i')

    # Controlled Variable
    x = m.Var(value=x_0, name='x')

    u0 = m.Var(value=0.0)
    i0 = m.Var(value=0.0)
    x0 = m.Var(value=x_0)

    for n in range(N+1):
        m.fix(u0, u_nlp[n], pos=n)
        m.fix(i0, i_nlp[n], pos=n)
        m.fix(x0, x_nlp[n], pos=n)

    f_var = m.Var(value=0, name='f_var')
    h_var = m.Var(value=0, name='h_var')
    obj_var = m.Var(value=0, name='obj_var')

    # Equations
    m.Equation(f_var == x.dt() - f_L((x, u, i), (x0, u0, i0)))
    m.Equation(h_var == h_L((x, u, i), (x0, u0, i0)))
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
    m.Equation(obj_var == F_GN((x-x_ref, u, i), (x0, u0, i0)))
    m.Minimize(obj_var) # F function

    m.options.IMODE = 6
    m.solve(disp=False)

    history = {}
    history['name_of_the_problem'] = name

    with open(m.path+'//results.json') as file:
        results = json.load(file)

    history['time'] = results['time']

    history['x'] = results['x']
    history['u'] = results['u']
    history['i'] = results['int_i']

    history['f'] = results['f_var']
    history['h'] = results['h_var']
    history['obj'] = results['obj_var']

    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_final'] = sum(history['obj']) # m.options.OBJFCNVAL

    return history

def linear(f, x, x0):
    """
    Linearization of the function f
    """
    f_ = lambda x: f(*x)
    x0 = [float(x0_i.value.__array__()) for x0_i in x0]
    f_grad = optimize.approx_fprime(x0, f_, 1e-6)
    delta = np.array(x) - np.array(x0)
    return f_(x0) + np.dot(f_grad, delta)

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
    x0 = [float(x0_i.value.__array__()) for x0_i in x0]
    f_grad = optimize.approx_fprime(x0, f_, 1e-6)
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
    print(f'Pos.semidef.: {pos_semidef(B_GN)}')
    return quadratic(f, x, x0, B_GN)

def show_results(history):
    """
    Function for displaying plots
    """
    time = history['time']
    x_0 = history['x_0']
    x_ref = history['x_ref']
    
    x = history['x']
    u = history['u']
    i = history['i']
    
    f = history['f']
    h = history['h']
    obj = history['obj']

    obj_final = history['obj_final']

    fig, axes = plt.subplots(3, 2, figsize=(15, 7), sharex=True)
    plt.suptitle(f'Objective value: {obj_final:.4f}')
    
    axes[0][0].step(time[:-1], u[1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[0][0].grid()
    axes[0][0].set_ylim(-0.1, 1.1)
    axes[0][0].set_ylabel('Control u')
    
    axes[1][0].step(time[:-1], i[1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[1][0].grid()
    axes[1][0].set_ylim(-0.1, 1.1)
    axes[1][0].set_ylabel('Control i')

    axes[2][0].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    axes[2][0].hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label='reference')
    axes[2][0].plot(time, x, '-o', c='r', markersize=5, linewidth=1, label='predicted trajectory')
    axes[2][0].legend()
    axes[2][0].grid()
    axes[2][0].set_xticks(time)
    axes[2][0].set_xticklabels(['' if i % 2 else t for i, t in enumerate(time)])
    axes[2][0].set_ylabel('State x')
    axes[2][0].set_xlabel('Time')
    
    axes[0][1].plot(time[:-1], f[1:], '-o')
    axes[0][1].set_ylabel('Value f')
    axes[0][1].grid()
    
    axes[1][1].plot(time[:-1], h[1:], '-o')
    axes[1][1].set_ylabel('Value h')
    axes[1][1].grid()

    axes[2][1].plot(time[:-1], obj[1:], '-o')
    axes[2][1].set_ylabel('Objective value')
    axes[2][1].set_xlabel('Time')
    axes[2][1].grid()
    
    plt.show()
    
def compare_final_obj(history_minlp, history_miqp):
    """
    """
    obj_minlp = history_minlp['obj_final']
    obj_miqp = history_miqp['obj_final']
    diff = obj_minlp - obj_miqp
    sign = 'positive' if diff > 0 else 'negative'
    resl = 'better' if diff > 0 else 'worse'

    print(f'MIQP  solution: {obj_miqp:.5f}')
    print(f'MINLP solution: {obj_minlp:.5f}')
    print(f'Difference: {diff:.5f} - {sign} => MIQP is {resl} than MINLP')
    
def compare_obj_trajectory(history_minlp, history_nlp, history_dist, history_miqp):
    """
    """
    plt.figure(figsize=(15, 4))
    plt.title('Objective value')
    plt.xlabel('Iteration')
    plt.ylabel('Value')

    plt.plot(history_minlp['obj'], c='blue', label='minlp')
    plt.plot(history_nlp['obj'], c='red', label='nlp')
#     plt.plot(history_dist['obj'], c='orange', label='dist')
    plt.plot(history_miqp['obj'], c='greenyellow', label='miqp')
    plt.legend()
    
    plt.show()
    
def compare_var_trajectory(history_minlp, history_miqp):
    """
    """
    fig, axes = plt.subplots(3, 1, figsize=(15, 7), sharex=True)

    time = history_minlp['time']

    axes[0].step(time[:-1], history_minlp['u'][1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[0].step(time[:-1], history_miqp['u'][1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel('Control u')

    axes[1].step(time[:-1], history_minlp['i'][1:], 'o-', markersize=5, linewidth=1.4, where='post', label='MINLP')
    axes[1].step(time[:-1], history_miqp['i'][1:], 'o-', markersize=5, linewidth=1.4, where='post', label='MIQP')
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_ylabel('Control i')
    axes[1].legend()

    # axes[2].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    # axes[2].hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label='reference')
    axes[2].plot(time, history_minlp['x'], '-o', markersize=5, linewidth=1, label='MINLP pred.trajectory')
    axes[2].plot(time, history_miqp['x'], '-o', markersize=5, linewidth=1, label='MIQP pred.trajectory')
    axes[2].legend()
    axes[2].set_ylabel('State x')
    axes[2].set_xlabel('Time')

    plt.show()