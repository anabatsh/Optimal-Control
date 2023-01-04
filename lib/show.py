import numpy as np
import matplotlib.pyplot as plt


def show_results(history):
    """
    Displaying plots
    """
    name = history['name_of_the_problem']
    time = history['time']
    x_0 = history['x_0']
    x_ref = history['x_ref']
 
    x = history['x']
    u = history['u']
    i = history['i']
    
    F = history['F']
    f = history['f']
    h = history['h']

    obj_value = history['obj_value']

    fig, axes = plt.subplots(3, 2, figsize=(15, 7), sharex=True)
    plt.suptitle(name)
    
    # -------------------------------------------------------------------------------------------
    axes[0][0].set_title('Control u')
    axes[0][0].step(time[:-1], u[1:], 'o-', markersize=4, linewidth=1.4, where='post')
    axes[0][0].set_ylim(-0.1, 1.1)
    axes[0][0].grid(alpha=0.5)

    # -------------------------------------------------------------------------------------------
    axes[1][0].set_title('Control i')
    axes[1][0].step(time[:-1], i[1:], 'o-', markersize=4, linewidth=1.4, where='post')
    axes[1][0].set_ylim(-0.1, 1.1)
    axes[1][0].grid(alpha=0.5)

    # -------------------------------------------------------------------------------------------
    axes[2][0].set_title('State x')
    axes[2][0].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    axes[2][0].hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label='reference')
    axes[2][0].plot(time, x, '-o', c='r', markersize=4, linewidth=1, label='predicted trajectory')
    axes[2][0].set_xticks(time)
    axes[2][0].set_xticklabels(['' if i % 2 else t for i, t in enumerate(time)])
    axes[2][0].set_xlabel('Time')
    axes[2][0].grid(alpha=0.5)
    axes[2][0].legend(loc=1)
    
    # -------------------------------------------------------------------------------------------
    axes[0][1].set_title('Equality function f')
    axes[0][1].plot(time[:-1], f[1:], '-o', markersize=4)
    axes[0][1].grid(alpha=0.5)
    
    # -------------------------------------------------------------------------------------------
    axes[1][1].set_title('Inequality function h')
    axes[1][1].plot(time[:-1], h[1:], '-o', markersize=4)
    axes[1][1].grid(alpha=0.5)

    # -------------------------------------------------------------------------------------------
    axes[2][1].set_title('Objective function F')
    axes[2][1].plot(time, F, '-o', markersize=4)
    axes[2][1].set_xlabel('Time')
    axes[2][1].grid(alpha=0.5)
    
    plt.show()
    # -------------------------------------------------------------------------------------------
    print(f'Objective value: {obj_value:.10f}')
    
def compare_results(history_minlp, history_nlp, history_gn, eps=1e-5):
    """
    Comparing total results
    """
    time = history_minlp['time']
    x_0 = history_minlp['x_0']
    x_ref = history_minlp['x_ref']
    
    x_minlp, u_minlp, i_minlp = [history_minlp[n] for n in ('x', 'u', 'i')]
    x_nlp, u_nlp, i_nlp = [history_nlp[n] for n in ('x', 'u', 'i')]
    x_gn, u_gn, i_gn = [history_gn[n] for n in ('x', 'u', 'i')]
    
    obj_minlp = history_minlp['obj_value']
    obj_nlp = history_nlp['obj_value']
    obj_gn = history_gn['obj_value']

    # -------------------------------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 7), gridspec_kw={'height_ratios': [2, 2, 3]})

    axes[0].step(time[:-1], u_minlp[1:], '--', c='blue', where='post', label='MINLP')
    axes[0].step(time[:-1], u_nlp[1:], c='black', where='post', label='NLP')
    axes[0].step(time[:-1], u_gn[1:], '--', c='red', where='post', label='GN-MIQP')
    axes[0].set_title('Control u')
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].grid(alpha=0.5)
    axes[0].legend(loc=1)

    axes[1].step(time[:-1], i_minlp[1:], '--', c='blue', where='post', label='MINLP')
    axes[1].step(time[:-1], i_nlp[1:], c='black', where='post', label='NLP')
    axes[1].step(time[:-1], i_gn[1:], '--', c='red', where='post', label='GN-MIQP')
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_title('Control i')
    axes[1].grid(alpha=0.5)
    axes[1].legend(loc=1)

#     axes[2].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    axes[2].hlines(x_ref, time[0], time[-1], colors='green', linestyles='dashed', label='reference')
    axes[2].plot(time, x_minlp, c='blue', label='MINLP')
    axes[2].plot(time, x_nlp, c='black', label='NLP')
    axes[2].plot(time, x_gn, c='red', label='GN-MIQP')
    axes[2].set_title('State x')
    axes[2].set_xlabel('Time')
    axes[2].set_xticks(time)
    axes[2].set_xticklabels(['' if i % 2 else t for i, t in enumerate(time)])
    axes[2].grid(alpha=0.5)
    axes[2].legend(loc=1)

    plt.show()
    
    # -------------------------------------------------------------------------------------------
    diff = obj_minlp - obj_gn
    if abs(diff) < eps:
        resl = 'equal to'
    else: 
        if diff > 0:
            resl = 'better than'
        else:
            resl = 'worse than'
        
    print('Objective value')
    print('-'*21)
    print(f'GEKKO MINLP : {obj_minlp:.5f}')
    print(f'        NLP : {obj_nlp:.5f}')
    print(f'    GN-MIQP : {obj_gn:.5f}')
    print('-'*21)
    print(f'GN-MIQP is {resl} GEKKO MINLP')