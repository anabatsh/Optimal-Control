import numpy as np
import matplotlib.pyplot as plt
import gekko
from scipy import optimize


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
    if type(x0[0]) == gekko.gk_variable.GKVariable:
        x0 = [float(x0_i.value.__array__()) for x0_i in x0]
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