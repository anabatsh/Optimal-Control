import numpy as np
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap
from gekko import GEKKO


def optimal_control_minimization(x_0, x_ref, time, f, F, integer=False):
    """
    params:
        x_0 - initial state
        x_ref - final reference state  
        time - time horizon
        f - differential equation
        F - objective function
        integer - integrality flag default False
    returns: 
        history: a dict with
            x - predicted state trjectory of x(t)
            i - best sequense of controls i(t)
            obj_val - corresponding objective value F(x, i)
        and some other items
    """
    m = GEKKO(remote=False)
    m.time = time
    m.options.SOLVER = 1 if integer else 3

    x = m.Var(value=x_0, name='x')
    i = m.Var(value=0.0, integer=integer, lb=0, ub=1, name='i') 

    m.Equation(x.dt() == f(x, i))
    m.Obj(F(x, i))

    m.options.IMODE = 6
    m.solve(disp=False)
    
    history = {}
    history['time'] = time
    history['x'] = x
    history['i'] = i
    history['obj_val'] = m.options.OBJFCNVAL
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    return history

def optimal_control_substituting(x_0, x_ref, time, f, F, const_i):
    """
    params:
        x_0 - initial state
        x_ref - final reference state  
        time - time horizon
        f - differential equation
        F - objective function
        const_i - fixed sequence of i(t)
    returns:
        history: a dict with
            x - predicted state trjectory of x(t)
            i - given sequense of controls i(t)
            obj_val - corresponding objective value F(x, i)
        and some other items
    """
    m = GEKKO(remote=False)
    m.time = time

    x = m.Var(value=x_0, name='x')
    i = m.Param(const_i, name='i') 
    obj_val = m.Var(value=0.0)
    
    m.Equation(x.dt() == f(x, i))
    m.Equation(obj_val == F(x, i))

    m.options.IMODE = 4
    m.solve(disp=False)

    history = {}
    history['time'] = time
    history['x'] = x
    history['i'] = i
    history['obj_val'] = sum(obj_val.VALUE)
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    return history

def show(history):
    """
    Displaying plots
    """
    time = history['time']
    x_0 = history['x_0']
    x_ref = history['x_ref']
    x = history['x']
    i = history['i']
    obj_val = history['obj_val']
    
    fig, axes = plt.subplots(2, 1, figsize=(6, 3), sharex=True)
    plt.subplots_adjust(hspace=0.3)
    
    axes[0].set_title('Control i', fontsize=10)
    axes[0].step(time[:-1], i[1:], 'o-', markersize=4, linewidth=1.4, where='post')
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].grid(alpha=0.5)
    
    axes[1].set_title('State x', fontsize=10)
    axes[1].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label=r'$x_0$')
    axes[1].hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label=r'$x_{ref}$')
    axes[1].plot(time, x, '-o', c='r', markersize=4, linewidth=1)
    axes[1].set_xticks(time)
    axes[1].set_xticklabels(['' if i % 2 else round(t, 2) for i, t in enumerate(time)])
    axes[1].set_xlabel('Time')
    axes[1].grid(alpha=0.5)
    axes[1].legend(loc=1)
    
    plt.show()
    
    print(f'Objective value = {obj_val:.6f}')
    
def show_X(X, x_0, x_ref, time, w, p):
    """
    """
    cmap = get_cmap('plasma')
    colors = cmap(w)
    time_labels = ['' if i % 2 else round(t, 2) for i, t in enumerate(time)]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4), height_ratios=[1, 3])
    plt.subplots_adjust(hspace=0.25)
    
    ax1.set_title('Probabilities p', fontsize=10)
#     ax1.bar(time, p, width=0.03)
    ax1.imshow([p], cmap='gray', vmin=0.0, vmax=1.0)
    ax1.set_yticks([])
    ax1.set_xticks(range(len(time)))
    ax1.set_xticklabels(time_labels)
    
    ax2.set_title('State x', fontsize=10)
    ax2.hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label=r'$x_0$')
    ax2.hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label=r'$x_{ref}$')
    for x, c in zip(X, colors):
        ax2.plot(time, x, '-o', c=c, markersize=4, linewidth=1)
    ax2.set_xticks(time)
    ax2.set_xticklabels(time_labels)
    ax2.set_xlabel('Time')
    ax2.grid(alpha=0.5)
    ax2.legend(loc=1)
    
    plt.show()