# Optimal-Control

There are two examples realized with [GEKKO](https://gekko.readthedocs.io/en/latest/):

---

#### Simple one: Mixed Integer Non Linear Programming problem:

* There is no time-horizon
* State and control variables are the same

$$
\min\limits_{x_1, x_2, x_3, x_4}
x_1 x_4 (x_1 + x_2 + x_3) + x_3 \\
x_1 x_2 x_3 x_4 \geq 25 \\
x^2_1 + x^2_2 + x^2_3 + x^2_4 = 40 \\
1 \geq x_1, x_2, x_3, x_4 \geq 5 \\
x_0 = (1, 5, 5, 1)
$$

Current results:

1. The algorithm (Gauss-Newton) has been implemented entirely. Everything works correctly. 



Current bottlenecks:

1. The GN algorithm doesn't work better than the existing method from GEKKO ([Branch and Bound](https://en.wikipedia.org/wiki/Branch_and_bound) based algorithm). 



What I plan to do:

1. Find more different examples of the problem and try the method.

---

#### Complicated one: Mixed Integer Non Linear Optimal Control problem:

* There is a time-horizon

$$
\min\limits_{\textbf{x}, \textbf{i}}
F(\textbf{x}, \textbf{i}), \quad
\text{such that} \;\;
\left\{ \begin{array}{l}
x(t_0) = x_0 \\
\dot{x}(t) = x^3(t) - i(t) \\
\textbf{i} \in P \cap \mathbb{Z}^{N} \\
\end{array} \right. \\

F(\textbf{x}, \textbf{i}) = 
\frac{1}{2} \int\limits^{T}_{t_0} (x(t) - x_{ref})^2 dt \\

N = 30, \ x_0 = 0.8, \ x_{ref} = 0.7
$$

Current results:

1. The first step (S1) of the tree-step GN algorithm has been implemented. 



Current bottlenecks:

1. It's hard to extract multidimensional gradients of GEKKO-type functions for the second step (S2) .



What I plan to do:

1. Compute each partial derivative independently. But how to do it correctly still doesn't clear enough. 

