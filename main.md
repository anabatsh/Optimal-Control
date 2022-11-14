## Optimal Control with GEKKO



### Optimal Control

Suppose we have

* $x$ - state variable
* $u$ - control variable
* $t$ - time variable $\in [t_0, t_f] = N$ - time horizon

**Continuous time**:

Cost function: $F(\cdot) = V_f(x(t_f)) + \int\limits_{t_0}^{t_f} l(x(t), u(t)) dt$

* State equation: $\dot{x} = f(x, u)$
* Path constraints: $h(x(t), u(t), t) \leq 0$
* Final constraint: $h_f(x(t_f)) \leq 0$


**Discrete time**:

In this case we can use an integer index $k$ instead of a real argument $t$:
$$
t_k \in [t_0, t_f] \mapsto k \in [0, N]: x(t_k) = x(k)
$$
 Cost function: $F(\cdot) = V_f(x(N)) + \sum\limits_{k=0}^{N-1} l(x(k), u(k))$

* State equation: $x^{+}$ or just $x(k+1) = f(x(k), u(k))$
* Path constraints: $h(x(k), u(k)) \leq 0$
* Final constraint: $h_f(x(N)) \leq 0$

In fact, by solving an optimal control problem on the computer we always solve its descitised version obtained with, for example, Runge-Kutta method. So hereinafter we will use only the discrete time formulation.

### Mixed-Integer Optimal Control

Suppose in addition to a real control variable $u$ we have an integer variable $i$:
* $x \in \mathbb{R}^{n_x}$ - state variable
* $u \in \mathbb{R}^{n_u}$ - continuous control variable
* $i \in \mathbb{Z}^{n_i}$ - integer control variable inside a bounded convex polyhedron P

And our objective function $F$ consists of a nonlinear least squares term $F1$ and nonlinear term $F_2$ - both are differentiable.

Lets note
* $\textbf{x} = (x(0), x(1), \dots, x(N))$
* $\textbf{u} = (u(0), u(1), \dots, u(N-1))$
* $\textbf{i} = (i(0), i(1), \dots, i(N-1))$

So we have **Mixed-Integer Optimal Control** problem ($\star$):
$$
\min\limits_{\textbf{x}, \textbf{u}, \textbf{i}}
{F(\textbf{x}, \textbf{u}, \textbf{i})}, \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k)) \\
h(x(k), u(k), i(k)) \leq 0, k < N \\
h_f(x(N)) \leq 0 \\
\textbf{i} \in P \\
\textbf{i} \in \mathbb{Z}^{N \cdot n_i}
\end{array} \right.
$$
where 
$$
\left\{ \begin{array}{l}
F(\textbf{x}, \textbf{u}, \textbf{i}) = 
V_f(x(N)) + \sum\limits_{k=0}^{N-1} l(x(k), u(k), i(k)) \\
\left. \begin{array}{l}
l(x, u, i) = 
\frac{1}{2} \|l_1(x, u, i)\|_2^2 + l_2(x, u, i) \\
V_f(x) = 
\frac{1}{2} \|V_1(x)\|_2^2 + V_2(x)
\end{array} \right\}
\Rightarrow
F(\textbf{x}, \textbf{u}, \textbf{i}) = 
\frac{1}{2} \|F_1(\textbf{x}, \textbf{u}, \textbf{i})\|_2^2 + 
F_2(\textbf{x}, \textbf{u}, \textbf{i}) \\
\end{array} \right.
$$


### Gauss-Newton algorithm

The proposal algorithm consists of three steps:

**S1**: solve the system without integrality constraint $\textbf{i} \in \mathbb{Z}^{n_i}$:
$$
(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*) = \arg\min\limits_{\textbf{x, u, i}}{F(\textbf{x, u, i})}, \quad
i \in P
$$
**S2**: approximate our continuous solution $(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*)$ by an integer $(\textbf{x}^{**}, \textbf{u}^{**}, \textbf{i}^{**})$

But firstly denote for a notation simplification: (from the article)

* $z = (\textbf{x}, \textbf{u}) \in \mathbb{R}^{n_z}$, where $n_z = (N+1) \cdot n_x + N \cdot n_u$ - continuous variable
* $y = \textbf{i} \in \mathbb{R}^{n_y}$, where $n_y = N \cdot n_i$ - integer variable

then we have **Mixed-Integer Non Linear Programming** problem ($\star\star$) instead of **Mixed-Integer Non Linear Optimal Control** one ($\star$):

$$
\min\limits_{y, z}{F(y, z)}, \quad
\left\{ \begin{array}{l}
G(y, z) = 0 
\;\; \text{instead of} \;
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k))) 
\end{array} \right. \\
H(y, z) \leq 0
\;\; \text{instead of} \;
\left\{ \begin{array}{l}
h(x(k), u(k), i(k)) \leq 0, k < N \\
h_f(x(N)) \leq 0
\end{array} \right. \\
y \in P \cap \mathbb{Z}^{n_y}
\end{array} \right.
$$
So, after the first step (**S1**) we have a continuous solution $(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*) = (z^*, y^*)$. 

And we want to approximate the continuous solution $y^*$ by the integer solution $y^{**}$:
$$
y^{**} = \arg\min\limits_{y}{d(y, y^*)}, \quad
y \in P \cup Z^{n_y}
$$
**Proposal Gauss-Newton algorithm**:

$d(y, y^*) = J_{GN}(y \mid y^*, z^*) - J_{NLP}(y^*)$, where

$J_{GN}(y \mid y^*, z^*) = J_{QP}(y \mid y^*, z^*, B_{GN}(y^*, z^*))$, where 

$
J_{QP}(y \mid y^*, z^*, B) =
\min\limits_{z}{F_{QP}(y, z \mid y^*, z^*, B)}, \quad
\left\{ \begin{array}{l}
G_L(y, z \mid y^*, z^*) = 0 \\
H_L(y, z \mid y^*, z^*) \leq 0 \\
\end{array} \right.
$ where

* $
    B_{GN}(y^*, z^*) = 
    \frac{\partial F_1}{\partial (y, z)}(y^*, z^*)
    (\frac{\partial F_1}{\partial (y, z)}(y^*, z^*))^T
  $

* $
    F_{QP}(y, z \mid y^*, z^*, B) = 
    F_L(y, z \mid y^*, z^*) + 
    \frac{1}{2} 
    \begin{bmatrix} y - y^* \\ z - z^* \end{bmatrix}^T
    B
    \begin{bmatrix} y - y^* \\ z - z^* \end{bmatrix}
  $
* $
   F_{L}(y, z \mid y^*, z^*) = 
   F(y^*, z^*) + \frac{\partial F}{\partial (y, z)}(y^*, z^*)
   ((y, z) - (y^*, z^*))
  $
* $
   G_{L}(y, z \mid y^*, z^*) = 
   G(y^*, z^*) + \frac{\partial G}{\partial (y, z)}(y^*, z^*)
   ((y, z) - (y^*, z^*))
  $
* $
   H_{L}(y, z \mid y^*, z^*) = 
   H(y^*, z^*) + \frac{\partial H}{\partial (y, z)}(y^*, z^*)
   ((y, z) - (y^*, z^*))
  $



**S3**: solve the NLP system with fixed variable $\textbf{i} = \textbf{i}^{**} = y^{**}$:
$$
(\textbf{x}^{***}, \textbf{u}^{***}, \textbf{i}^{**}) = \arg\min\limits_{\textbf{x, u}}{F(\textbf{x}, \textbf{u}, \textbf{i}^{**})}
$$


## GEKKO adaptation

Firstly lets rewrite MINLP ($\star$) and simplify it a little: 

1. Add empty variables $u(N), i(N)$. Their values don't matter, they need only to make $\textbf{u}$ and $\textbf{i}$ same size as $\textbf{x}$. 
   * $\textbf{x} = (x(0), x(1), \dots, x(N))$
   * $\textbf{u} = (u(0), u(1), \dots, u(N))$
   * $\textbf{i} = (i(0), i(1), \dots, i(N))$
2. Assume that $h_f(x(N)) = h(x(N), u(N), i(N))$.
3. Assume that $V_f(x(N)) = l(x(N), u(N), i(N))$.
4. Rewrite $F(\textbf{x}, \textbf{u}, \textbf{i})$ as a sum of $F(x(k), u(k), i(k))$, where $F(x, u, i) = \frac{1}{2} \|F_1(x, u, i)\|_2^2 + F_2(x, u, i)$.

Then our MINL problem looks like this
$$
\min\limits_{\textbf{x}, \textbf{u}, \textbf{i}}
{F(\textbf{x}, \textbf{u}, \textbf{i})}, \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k)) \\
h(x(k), u(k), i(k)) \leqslant 0 \\
\textbf{i} \in P \cap \mathbb{Z}^{N \cdot n_i}
\end{array} \right.
$$
where $F(\textbf{x}, \textbf{u}, \textbf{i}) = \sum\limits_{k=0}^{N} F(x(k), u(k), i(k))$ and $F(x, u, i) = \frac{1}{2} \|F_1(x, u, i)\|_2^2 + F_2(x, u, i)$.

But in practice of working with GEKKO it will be better to return to an Optimal Control formulation:
$$
\textbf{i}^{**} = 
\arg\min\limits_{\textbf{i}}{
d(\textbf{i}, \textbf{i}^{*})
}, \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f_L(x(k), u(k), i(k)) \\
h_L(x(k), u(k), i(k)) \leq 0, k < N \\
h_{fL}(x(N)) \leq 0 \\
\textbf{i} \in P \cap \mathbb{Z}^{N \cdot n_i}
\end{array} \right.
$$
However, it's not clear, why we can replace functions depand of $\textbf{x},\textbf{u},\textbf{i}$ entirely by functions depand only of $(x(k), u(k), i(k))$ on each time step. But in the Appendix we prove that we can. 



## Example

We will consider the problem from the article:
$$
\begin{gather}
\begin{array}{rll}
\text{continuous time}: & 
    \dot{x}(t) = f(x(t), u(t), i(t)) = x^3(t) - i(t) \\
\text{discrete time}: & 
    x(k+1) = \hat{f}(x(k), u(k), i(k)) = \text{Runge-Kutta-4}\ (f)
\end{array} \\ \\
\text{objetive function}:
F(\textbf{x}, \textbf{u}, \textbf{i}) = 
\frac{1}{2} \sum\limits^{N}_{k=0} 
(x(k) - x_{ref})^2 \\ \\
\textbf{MINLP}: 
\min\limits_{\textbf{x}, \textbf{u}, \textbf{i}}
F(\textbf{x}, \textbf{u}, \textbf{i}), \quad
\text{such that} \;\;
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k)) \\
\textbf{i} \in P \cap \mathbb{Z}^{N} \\
\end{array} \right.
\end{gather}
$$

$$
P = 
\left\{ \textbf{i} \in [0, 1]^N \ \bigg| \;\; 
\begin{align} 
    & i(k) \geq i(k-1) - i(k-2) \\
    & i(k) \geq i(k-1) - i(k-3) 
\end{align} \right\}
$$

Additional parameters: $N = 30, \ x_0 = 0.8, \ x_{ref} = 0.7$



Some remarks:

1. We can consider $F(\textbf{x}, \textbf{u}, \textbf{i})$ as a sum of $F(x(k), u(k), i(k))$, where $F(x, u, i) = \frac{1}{2} \|F_1(x, u, i)\|_2^2$ and $F_1(x, u, i) = (x - x_{ref}, 0, 0)$.
2. Instead of Runge-Kutta methods we will use the collocations method. It's already implemented in GEKKO.
3. Firstly we will try $f(x, i) = x - i$ instead of $f(x, i) = x^3 - i$. Because in this case the Gauss-Newton system on the second step should be equal to the original system (linearization of $f$ is equal to $f$ if $f$ is a linear function initially and quadratic approximation of $F$ is equal to $F$ if $F$ is a quadratic function initially). 



### Experiment 1





## Example 2



## Appendix

Consider $G$ function:
$$
G(y, z) = 0 
\;\; \text{instead of} \;
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k))) 
\end{array} \right.
$$
It means that $G(y, z)$ looks like:

$$
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(1) = f(x(0), u(0), i(0))) \\
x(2) = f(x(1), u(1), i(1))) \\
\dots \\
% x(N) = f(x(N-1), u(N-1), i(N-1))) \\
\end{array} \right.
\iff
\textbf{x} = 
\begin{bmatrix}
x(0) \\ x(1) \\ x(2) \\ \dots
\end{bmatrix}
=
\begin{bmatrix}
x_0 \\
f(x(0), u(0), i(0))) \\
f(x(1), u(1), i(1))) \\
\dots \\
% f(x(N-1), u(N-1), i(N-1))) \\
\end{bmatrix}
= \textbf{f}(\textbf{x}, \textbf{u}, \textbf{i})
\Rightarrow
G(y, z) = 
G(\textbf{x}, \textbf{u}, \textbf{i}) = 
\textbf{x} - \textbf{f}(\textbf{x}, \textbf{u}, \textbf{i})
$$
Thus
$$
\frac{\partial G}{\partial (y, z)} = 
(
    \frac{\partial G}{\partial y}, 
    \frac{\partial G}{\partial z}
) = 
(
    \frac{\partial G}{\partial \textbf{i}}, 
    \frac{\partial G}{\partial \textbf{u}},
    \frac{\partial G}{\partial \textbf{x}}
) = 
(
    -\frac{\partial \textbf{f}}{\partial \textbf{i}}, 
    -\frac{\partial \textbf{f}}{\partial \textbf{u}},
    I - \frac{\partial \textbf{f}}{\partial \textbf{x}}
)
$$
where
$$
\begin{split}
& \frac{\partial \textbf{f}}{\partial \textbf{i}} = 
\big[ 
\frac{\partial \textbf{f}(k)}{\partial i(s)} 
\big]^{(N+1) \times N}
k = \overline{0, N}, s = \overline{0, N-1} \;
\Rightarrow
\frac{\partial \textbf{f}}{\partial \textbf{i}} = 
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial i}(i(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial i}(i(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial i}(i(2)) & \dots \\
\dots \\
\end{bmatrix} \\
& \frac{\partial \textbf{f}}{\partial \textbf{u}} = 
\big[ 
\frac{\partial \textbf{f}(k)}{\partial u(s)} 
\big]^{(N+1) \times N}
k = \overline{0, N}, s = \overline{0, N-1}
\Rightarrow
\frac{\partial \textbf{f}}{\partial \textbf{u}} = 
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial u}(u(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial u}(u(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial u}(u(2)) & \dots \\
\dots \\
\end{bmatrix} \\
& \frac{\partial \textbf{f}}{\partial \textbf{x}} = 
\big[ 
\frac{\partial \textbf{f}(k)}{\partial x(s)} 
\big]^{(N+1) \times (N+1)}
k = \overline{0, N}, s = \overline{0, N} \
\Rightarrow
\frac{\partial \textbf{f}}{\partial \textbf{x}} = 
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial x}(x(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial x}(x(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial x}(x(2)) & \dots \\
\dots \\
\end{bmatrix}
\end{split}
$$
Hence 
$$
d G =
(-\frac{\partial \textbf{f}}{\partial \textbf{i}}) d \textbf{i} +
(-\frac{\partial \textbf{f}}{\partial \textbf{u}}) d \textbf{u} +
(I - \frac{\partial \textbf{f}}{\partial \textbf{x}}) d \textbf{x} 
=\\
-
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial i}(i(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial i}(i(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial i}(i(2)) & \dots \\
\dots \\
\end{bmatrix} 
\begin{bmatrix}
di(0) \\ di(1) \\ di(2) \\ \dots
\end{bmatrix}
- 
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial u}(u(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial u}(u(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial u}(u(2)) & \dots \\
\dots \\
\end{bmatrix} 
\begin{bmatrix}
du(0) \\ du(1) \\ du(2) \\ \dots
\end{bmatrix}
-
\begin{bmatrix}
0 & 0 & 0 & \dots \\
\frac{\partial f}{\partial x}(x(0)) & 0 & 0 & \dots \\
0 & \frac{\partial f}{\partial x}(x(1)) & 0 & \dots \\
0 & 0 & \frac{\partial f}{\partial x}(x(2)) & \dots \\
\dots \\
\end{bmatrix}
\begin{bmatrix}
dx(0) \\ dx(1) \\ dx(2) \\ \dots
\end{bmatrix}
+ 
\begin{bmatrix}
dx(0) \\ dx(1) \\ dx(2) \\ \dots
\end{bmatrix}
$$

It means that
$$
d G = 
-
\begin{bmatrix}
0 \\ 
\frac{\partial f}{\partial i}(i(0)) \ di(0) \\
\frac{\partial f}{\partial i}(i(1)) \ di(1) \\
\frac{\partial f}{\partial i}(i(2)) \ di(2) \\
\dots \\
\end{bmatrix}
-
\begin{bmatrix}
0 \\ 
\frac{\partial f}{\partial u}(u(0)) \ du(0) \\
\frac{\partial f}{\partial u}(u(1)) \ du(1) \\
\frac{\partial f}{\partial u}(u(2)) \ du(2) \\
\dots \\
\end{bmatrix}
-
\begin{bmatrix}
0 \\ 
\frac{\partial f}{\partial x}(x(0)) \ dx(0) \\
\frac{\partial f}{\partial x}(x(1)) \ dx(1) \\
\frac{\partial f}{\partial x}(x(2)) \ dx(2) \\
\dots \\
\end{bmatrix}
+
\begin{bmatrix}
dx(0) \\ dx(1) \\ dx(2) \\ \dots
\end{bmatrix}
$$
Thus
$$
G_{L}(y, z \mid y^*, z^*) = 
G(y^*, z^*) + \frac{\partial G}{\partial (y, z)}(y^*, z^*)((y, z) - (y^*, z^*)) = 
G^* + dG^* = \\
\underbrace{
    \begin{bmatrix}
    x^*(0) \\ x^*(1) \\ x^*(2) \\ \dots
    \end{bmatrix}
    -
    \begin{bmatrix}
    x_0 \\
    f(x^*(0), u^*(0), i^*(0))) \\
    f(x^*(1), u^*(1), i^*(1))) \\
    \dots \\
    \end{bmatrix}
}_{G^*}
\
\underbrace{
    -
    \begin{bmatrix}
    0 \\ 
    \frac{\partial f}{\partial i}(i(0)) \ (i(0) - i^*(0)) \\
    \frac{\partial f}{\partial i}(i(1)) \ (i(1) - i^*(1)) \\
    \dots \\
    \end{bmatrix}
    -
    \begin{bmatrix}
    0 \\ 
    \frac{\partial f}{\partial u}(u(0)) \ (u(0) - u^*(0)) \\
    \frac{\partial f}{\partial u}(u(1)) \ (u(1) - u^*(1)) \\
    \dots \\
    \end{bmatrix}
    -
    \begin{bmatrix}
    0 \\ 
    \frac{\partial f}{\partial x}(x(0)) \ (x(0) - x^*(0)) \\
    \frac{\partial f}{\partial x}(x(1)) \ (x(1) - x^*(1)) \\
    \dots \\
    \end{bmatrix}
    +
    \begin{bmatrix}
    (x(0) - x^*(0)) \\ 
    (x(1) - x^*(1)) \\ 
    (x(2) - x^*(2)) \\ 
    \dots
    \end{bmatrix}
}_{dG^*}
$$
So, if we gonna use $G_L$ instead of $G$ in the corresponding programming problem, we should use these constraints in the original optimal control problem:
$$
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x^*(k), u^*(k), i^*(k))) 

+ \frac{\partial f}{\partial i}(i(k)) \ (i(k) - i^*(k))
+ \frac{\partial f}{\partial u}(u(k)) \ (u(k) - u^*(k))
+ \frac{\partial f}{\partial x}(x(k)) \ (x(k) - x^*(k))
  \end{array} \right.
$$
It's easy to see that the right part of the equation is the linearization of the function f:

$$
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f_L(x(k), u(k), i(k)) \mid x^*(k), u^*(k), i^*(k)))
\end{array} \right.,
\quad
f_L(x, u, i \mid x^*, u^*, i^*) =
f(x^*, u^*, i^*) +
\frac{\partial f}{\partial (x, u, i)}((x, u, i) - (x^*, u^*, i^*))
$$
Obviously, there is a same situation with the inequality constraint function $H$: if we gonna use $H_L$ instead of $H$ in the programming problem, we should use $h_L, h_{F, L}$ instead of $h$ in the optimal control problem:

$$
\left\{ \begin{array}{l}
h_L(x(k), u(k), i(k)) \mid x^*(k), u^*(k), i^*(k))) \leqslant 0, k < N \\
h_{F, L}(x(N)) \leqslant 0
\end{array} \right.,
\quad
\left\{ \begin{array}{l}
h_L(x, u, i \mid x^*, u^*, i^*) =
h(x^*, u^*, i^*) +
\frac{\partial h}{\partial (x, u, i)}((x, u, i) - (x^*, u^*, i^*)) \\
h_{F, L}(x \mid x^*) =
h_F(x^*, u^*, i^*) +
\frac{\partial h_F}{\partial x}(x - x^*)
\end{array} \right.
$$
