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
    
def indicatr(x, lamb):
    """
    Indicator function of the top-s best (minimal) values:
    If x_i from x is one of the s minimal values of x, we 
    return 1 for this element and 0 otherwise
    params: 
        x - 1D real array 
        lamb - proportion of the best values s.t s = lamb% of x
    returns:
        w - binary vector with the indicator function values
    """
    top_s = int(lamb * len(x))
    w = (x <= np.sort(x)[top_s-1]).astype(np.float64)
    return w

def exponent(x, loc=0.0, lamb=1.0):
    """
    Exponent probability distribution function
    params: 
        x - 1D real array 
        lamb - parameter of the distribution
    returns:
        w - real vector with the corresponding pdf values
    """
    x_loc = x - loc
    w = np.where(x_loc >= 0, lamb * np.exp(-lamb * x_loc), 0)
    return w

def sample(n_samples, probabilities):
    """
    Sample K binary vectors from q(p_1, ..., p_{T-1})
    params: 
        n_samples = K - number of samples
        probabilities - parameters p_1 ... p_{T-1}
    returns:
        I - samples
    """
    T = len(probabilities)
    I = np.random.rand(n_samples, T)
    I = (I <= probabilities).astype(np.int32)
    return I

def optimal_control_custom(
    x_0, x_ref, time, f, F, n_samples, n_steps, mode='top-s', verbose=0, update=True, **kwargs):
    """
    params:
        x_0 - initial state
        x_ref - final reference state  
        time - time horizon
        f - differential equation
        F - objective function
        n_samples = K - number of samples
        mode - way of calculating weights of the samples
            'top-s' : kwargs.lamb - proportion of the best solutions
            'expon' : kwargs.lamb - parameter of the exponential distribution
        n_steps = N - number of steps
        verbose - output setting
            0 - show solutions
            1 - print objective values only
        update - True if the best solution is remembered
    returns:
        history: a dict with
            x - predicted state trjectory of x(t)
            i - given sequense of controls i(t)
            obj_val - corresponding objective value F(x, i)
        and some other items
    """
    best_sol = {'step' : 0, 'obj_val' : np.inf, 'x': None, 'i': None}
    
    p = np.ones_like(time) * 0.5
    history = {'p_trace' : [p], 'obj_val_trace': []}

    if mode == 'top-s':
        w_func = indicatr
    else:
        w_func = exponent
    
    # main iteration loop : 6
    for step in range(1, n_steps+1):
        X = []
        Obj = []
        
        # sample and substitute solutions : 1-3
        I = sample(n_samples, p)
        for const_i in I:
            history_i = optimal_control_substituting(x_0, x_ref, time, f, F, const_i)
            X.append(history_i['x'])
            Obj.append(history_i['obj_val'])
        Obj = np.array(Obj)
        min_id = np.argmin(Obj)
        curr_sol = {'step' : step, 'obj_val' : Obj[min_id], 'x': X[min_id], 'i': I[min_id]}
        if update and curr_sol['obj_val'] < best_sol['obj_val'] or not update:
            best_sol = curr_sol
            
        # find best solutions : 4
        # update probabilities : 5
        w = w_func(Obj, **kwargs)
        
        print('-'*80)
        c, b = curr_sol['obj_val'], best_sol['obj_val']
        print(f'Step {step}/{n_steps} | Objective Value (current) {c:.5f} | Objective Value (total) {b:.5f}')
        if verbose == 0:
            show_X(X, x_0, x_ref, time, w, p)

        p = np.average(I, axis=0, weights=w)
#         print('w:', np.around(w, 3))
#         print('p:', np.around(p, 3))
        
        history['obj_val_trace'].append(Obj)
        history['p_trace'].append(p)
        
    history['time'] = time
    history['x'] = best_sol['x']
    history['i'] = best_sol['i']
    history['obj_val'] = best_sol['obj_val']
    history['x_0'] = x_0
    history['x_ref'] = x_ref
    history['obj_val_trace'] = np.array(history['obj_val_trace'])
    history['p_trace'] = np.array(history['p_trace'])
    return history