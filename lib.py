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
    u = m.Var(value=0.0, lb=0, ub=1, name='u')
    
    # Manipulated variable: integer
    i = m.Var(value=0.0, integer=integer, lb=0, ub=1, name='i')

    # Controlled Variable
    x = m.Var(value=x_0, name='x')

    # Equations
    f_var = m.Var(value=0.0, name='f_var')
    h_var = m.Var(value=0.0, name='h_var')
    m.Equation(f_var == x.dt() - f(x, u, i)) # f function
    m.Equation(h_var == h(x, u, i)) # f function
    m.Equation(f_var == 0)
    m.Equation(h_var <= 0)
    
    # Objective function
    obj_var = m.Var(value=0.0, name='obj_var')
    m.Equation(obj_var == F(x - x_ref, u, i)) # F function
    m.Obj(obj_var) 

    m.options.IMODE = 6
    m.solve(disp=False)
    
    history = {}
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
    history['obj_final'] = m.options.OBJFCNVAL
    
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
    
    axes[0][1].plot(time[:-1], f[1:])
    axes[0][1].set_ylabel('Value f')
    
    axes[1][1].plot(time[:-1], h[1:])
    axes[1][1].set_ylabel('Value h')

    axes[2][1].plot(time[:-1], obj[1:])
    axes[2][1].set_ylabel('Objective value')
    axes[2][1].set_xlabel('Time')
    
    plt.show()