import numpy as np
import matplotlib.pyplot as plt


def show_results(history):
    """
    Function for displaying plots
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
    axes[0][0].step(time[:-1], u[1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[0][0].grid()
    axes[0][0].set_ylim(-0.1, 1.1)
    
    # -------------------------------------------------------------------------------------------
    axes[1][0].set_title('Control i')
    axes[1][0].step(time[:-1], i[1:], 'o-', markersize=5, linewidth=1.4, where='post')
    axes[1][0].grid()
    axes[1][0].set_ylim(-0.1, 1.1)

    # -------------------------------------------------------------------------------------------
    axes[2][0].set_title('State x')
    axes[2][0].hlines(x_0, time[0], time[-1], colors='r', linestyles='dashed', label='initial')
    axes[2][0].hlines(x_ref, time[0], time[-1], colors='g', linestyles='dashed', label='reference')
    axes[2][0].plot(time, x, '-o', c='r', markersize=5, linewidth=1, label='predicted trajectory')
    axes[2][0].set_xticks(time)
    axes[2][0].set_xticklabels(['' if i % 2 else t for i, t in enumerate(time)])
    axes[2][0].set_xlabel('Time')
    axes[2][0].legend()
    axes[2][0].grid()
    
    # -------------------------------------------------------------------------------------------
    axes[0][1].set_title('Equality function f')
    axes[0][1].plot(time[:-1], f[1:], '-o')
    axes[0][1].grid()
    
    # -------------------------------------------------------------------------------------------
    axes[1][1].set_title('Inequality function h')
    axes[1][1].plot(time[:-1], h[1:], '-o')
    axes[1][1].grid()

    # -------------------------------------------------------------------------------------------
    axes[2][1].set_title('Objective function F')
    axes[2][1].plot(time[:-1], F[1:], '-o')
    axes[2][1].set_xlabel('Time')
    axes[2][1].grid()
    
    plt.show()
    # -------------------------------------------------------------------------------------------
    print(f'Objective value: {obj_value:.10f}')
    
def compare_final_obj(history_minlp, history_miqp):
    """
    """
    obj_minlp = history_minlp['obj_value']
    obj_miqp = history_miqp['obj_value']
    diff = obj_minlp - obj_miqp
    sign = 'zero'
    resl = 'not worse'
    if diff > 0:
        sign = 'positive'
        resl = 'better'
    if diff < 0: 
        sign = 'negative'
        resl = 'worse'
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

    plt.plot(history_minlp['F'], c='blue', label='minlp')
    plt.plot(history_nlp['F'], c='red', label='nlp')
#     plt.plot(history_dist['F'], c='orange', label='dist')
    plt.plot(history_miqp['F'], c='greenyellow', label='miqp')
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