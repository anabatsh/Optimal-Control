import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO
import json


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
    name=''         # name of the problem
):
    """
    Function for MINL Optimal Control solving
    """
    m = GEKKO()
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
    u = m.Var(value=0.5, lb=0, ub=1, name='u')
    
    # Manipulated variable: integer
    i = m.Var(value=0.5, integer=integer, lb=0, ub=1, name='i')

    # Controlled Variable
    x = m.Var(value=x_0, name='x')

    # Equations
    m.Equation(f(x, u, i) == 0) # f function
    m.Equation(h(x, u, i) <= 0) # H function

    # Objective function
    m.Obj(F(x - x_ref, u, i)) # F function

    m.options.IMODE = 6
    m.solve(disp=False)
    
    history = {}
    with open(m.path+'//results.json') as f:
        results = json.load(f)
    history['time'] = results['time']
    history['x'] = results['x']
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['u'] = results['u']
    history['i'] = results['int_i' if integer else 'i']
    history['obj'] = m.options.OBJFCNVAL
    
    return history

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
    obj = history['obj']

    plt.figure(figsize=(10, 7))
    plt.suptitle(f'Objective value: {obj:.4f}')
    
    plt.subplot(3, 1, 1)
    plt.step(time, u, 'o-', 
             markersize=5, linewidth=1.4,
             where='post')
    plt.xticks([])
    plt.ylabel('Control u')
    
    plt.subplot(3, 1, 2)
    plt.step(time, i, 'o-', 
             markersize=5, linewidth=1.4,
             where='post')
    plt.xticks([])
    plt.ylabel('Control i')

    plt.subplot(3, 1, 3)
    plt.hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    plt.hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label='reference')
    plt.plot(time, x, '-o', c='r',
             markersize=5, linewidth=1,
             label='predicted trajectory')
    plt.legend(loc=4)
    plt.ylabel('State x')
    plt.xlabel('Time')

    plt.show()