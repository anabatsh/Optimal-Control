## Optimal Control with GEKKO



### Optimal Control

Suppose we have

* $x$ - state variable $\in \mathbb{R}^{n_x}$
* $u$ - control variable $\in \mathbb{R}^{n_u}$
* $k$ - discrete time variable $\in \{0, \dots, N\}$, where $N$ $\in \mathbb{N}$ is a time horizon

<img src="/Users/anabatsh/Library/Application Support/typora-user-images/image-20230105155142721.png" alt="image-20230105155142721" style="zoom:60%;" />

Define

* $\textbf{x} = (x(0), x(1), \dots, x(N))$
* $\textbf{u} = (u(0), u(1), \dots, u(N-1))$

Then the **Optimal Control Problem** is formulated as follows:
$$
F(\textbf{x}, \textbf{u}) \rightarrow \min\limits_{\textbf{x}, \textbf{u}} \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k)) \\
h(x(k), u(k)) \leq 0, k < N \\
\widehat{h}(x(N)) \leq 0 \\
\end{array} \right.
$$
where

* $F(\cdot) = V_f(x(N)) + \sum\limits_{k=0}^{N-1} l(x(k), u(k))$ - cost function
* $x(k+1) = f(x(k), u(k))$ - state equation

If $f$ is linear, then the problem is called **Linear Optimal Control**. Otherwise it's called **Non-Linear Optimal Control**.

* $h(x(k), u(k)) \leq 0$ - path constraints
* $\widehat{h}(x(N)) \leq 0$ - final constraint



### Mixed-Integer Optimal Control

Suppose in addition to a real control variable $u$ we have an integer control variable $i$:
* $x \in \mathbb{R}^{n_x}$ - state variable
* $u \in \mathbb{R}^{n_u}$ - continuous control variable
* $i \in \mathbb{Z}^{n_i}$ - integer control variable inside a bounded convex polyhedron P



Define

* $\textbf{x} = (x(0), x(1), \dots, x(N))$
* $\textbf{u} = (u(0), u(1), \dots, u(N-1))$
* $\textbf{i} = (i(0), i(1), \dots, i(N-1))$

Then the **Mixed-Integer Optimal Control** problem is formulated as follows:
$$
F(\textbf{x}, \textbf{u}, \textbf{i}) 
\rightarrow 
\min\limits_{\textbf{x}, \textbf{u}, \textbf{i}}
\quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k)) \\
h(x(k), u(k), i(k)) \leq 0, k < N \\
\widehat{h}(x(N)) \leq 0 \\
\textbf{i} \in P \\
\textbf{i} \in \mathbb{Z}^{N \cdot n_i}
\end{array} \right.
$$
Hereinafter we will assume that the objective terms consist of a nonlinear least squares term $\frac{1}{2}\|F_1(\cdot)\|^2_2$ and a nonlinear term $F_2(\cdot)$ - both are differentiable. Thus,
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
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i(k)) \\
h(x(k), u(k), i(k)) \leq 0, k < N \\
\widehat{h}(x(N)) \leq 0 \\
\textbf{i} \in P
\end{array} \right.
$$
**S2**: approximate our continuous solution $\textbf{i}^*$ by an integer $\textbf{i}^{**}$ obtained as follows:
$$
\textbf{i}^{**} = 
\arg\min\limits_{\textbf{i}}{
d(\textbf{i}, \textbf{i}^{*})
}, \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f_L(x(k), u(k), i(k)) \\
h_L(x(k), u(k), i(k)) \leqslant 0 \\
\widehat{h}_L(x(k)) \leqslant 0 \\
\textbf{i} \in P \cap \mathbb{Z}^{N \cdot n_i}
\end{array} \right.
$$
**Proposal Gauss-Newton algorithm - key part**:

* $d(\textbf{i}, \textbf{i}^*) = J_{GN}(\textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*) - J_{NLP}(\textbf{i}^*)$ - distance function
* $J_{NLP}(\textbf{i}^*) = F(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*)$

Gauss-Newton approximation part:

* $J_{GN}(\textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*) =
  \min\limits_{\textbf{x}, \textbf{u}}{F_{GN}(\textbf{x}, \textbf{u}, \textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*)}$

* $F_{GN}(\textbf{x}, \textbf{u}, \textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*) = 
   F_{QP}(\textbf{x}, \textbf{u}, \textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*, B_{GN}(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*))$
* $F_{QP}(\textbf{x}, \textbf{u}, \textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*, B) = 
  F_L(\textbf{x}, \textbf{u}, \textbf{i} \mid \textbf{x}^*, \textbf{u}^*, \textbf{i}^*) + 
  \frac{1}{2} 
  \begin{bmatrix} 
  \textbf{x} - \textbf{x}^* \\ 
  \textbf{u} - \textbf{u}^* \\ 
  \textbf{i} - \textbf{i}^* 
  \end{bmatrix}^T
  B
  \begin{bmatrix} 
  \textbf{x} - \textbf{x}^* \\ 
  \textbf{u} - \textbf{u}^* \\ 
  \textbf{i} - \textbf{i}^* 
  \end{bmatrix}$​
* $B_{GN}(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*) = 
  \frac{\partial F_1}{\partial (\textbf{x}, \textbf{u}, \textbf{i})}(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*)
  \frac{\partial F_1}{\partial (\textbf{x}, \textbf{u}, \textbf{i})}(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*)^T$

Linearizations:

* $F_{L}(
  \textbf{x}, \textbf{u}, \textbf{i} \mid 
  \textbf{x}^*, \textbf{u}^*, \textbf{i}^*) = 
  F(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*) + 
  \frac{\partial F}{\partial (\textbf{x}, \textbf{u}, \textbf{i})}(\textbf{x}^*, \textbf{u}^*, \textbf{i}^*)
  ((\textbf{x}, \textbf{u}, \textbf{i}) - (\textbf{x}^*, \textbf{u}^*, \textbf{i}^*))$
* $f_{L}(x, u, i \mid x^*, u^*, i^*) = 
  f(x^*, u^*, i^*) + 
  \frac{\partial f}{\partial (x, u, i)}(x^*, u^*, i^*)
  ((x, u, i) - (x^*, u^*, i^*))$
* $h_{L}(x, u, i \mid x^*, u^*, i^*) = 
  h(x^*, u^*, i^*) + 
  \frac{\partial h}{\partial (x, u, i)}(x^*, u^*, i^*)
  ((x, u, i) - (x^*, u^*, i^*))$
* $\widehat{h}_{L}(x \mid x^*) = 
  \widehat{h}(x^*) + 
  \frac{\partial \widehat{h}}{\partial x}(x^*)(x - x^*)$



**S3**: solve the NLP system with fixed variable $\textbf{i} = \textbf{i}^{**}$:
$$
(\textbf{x}^{***}, \textbf{u}^{***}, \textbf{i}^{**}) = \arg\min\limits_{\textbf{x, u}}{F(\textbf{x}, \textbf{u}, \textbf{i}^{**})}
, \quad
\left\{ \begin{array}{l}
x(0) = x_0 \\
x(k+1) = f(x(k), u(k), i^{**}(k)) \\
h(x(k), u(k), i^{**}(k)) \leq 0, k < N \\
\widehat{h}(x(N)) \leq 0 \\
\end{array} \right.
$$

## Example

We will consider the problem from the article:
$$
\begin{gather*}
\begin{array}{rll}
\text{continuous time}: & 
    \dot{x}(t) = f_c(x(t), u(t), i(t)) = x^3(t) - i(t) \\
\text{discrete time}: & 
    x(k+1) = f(x(k), u(k), i(k)) = \text{Runge-Kutta-4}\ (f_c)
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
\end{gather*}
$$

$$
P = 
\left\{ 
\textbf{i} \in [0, 1]^N \ \bigg| \;
\begin{align*} 
    & i(k) \geq i(k-1) - i(k-2) \\
    & i(k) \geq i(k-1) - i(k-3) 
\end{align*} \right\}
$$

Additional parameters: $N = 30, \ x_0 = 0.8, \ x_{ref} = 0.7$

Some remarks:

1. We can consider $F(\textbf{x}, \textbf{u}, \textbf{i})$ as a sum of $F(x(k), u(k), i(k))$, where $F(x, u, i) = \frac{1}{2} \|F_1(x, u, i)\|_2^2$ and $F_1(x, u, i) = (x - x_{ref}, 0, 0)$.
2. Instead of Runge-Kutta methods we will use the collocations method. It's already implemented in GEKKO.
3. Firstly we will try $f(x, i) = x - i$ instead of $f(x, i) = x^3 - i$. Because in this case the Gauss-Newton system on the second step should be equal to the original system (linearization of $f$ is equal to $f$ if $f$ is a linear function initially and quadratic approximation of $F$ is equal to $F$ if $F$ is a quadratic function initially). 

### Results

#### Experiment 1

1. $F$ is square
2. $f$ is linear ($x - i$)

![1](/Users/anabatsh/Desktop/Optimal-Control/1.png)

```
Objective value
---------------------
GEKKO MINLP : 0.01661
        NLP : 0.01213
    GN-MIQP : 0.01660
---------------------
GN-MIQP is equal to GEKKO MINLP
```

In this case the results should be equal because $F_{GN}$ coincides with $F$ and $f_L$ coincides with $f$.



#### Experiment 2

1. $F$ is square
2. $f$ is non-linear ($x^3 - i$)

![2](/Users/anabatsh/Desktop/Optimal-Control/2.png)

We can see that the red line (on the bottom state graph) corresponded to Gauss-Newton method increases, moving away from the reference value. Since the objective function $F$ is square, $F_{GN}$ coincides with it. So, the only reason the solutions differ is because of the function $f_{L}$ doesn't coincide with $f$. Not surprisingly, because approximating a nonlinear function with a linear one leads to errors.

```
Objective value
---------------------
GEKKO MINLP : 0.01892
        NLP : 0.00369
    GN-MIQP : 0.09791
---------------------
GN-MIQP is worse than GEKKO MINLP
```

Moreover, for this case the authors propose an exact solution. Let's substitute this solution.

![3](/Users/anabatsh/Desktop/Optimal-Control/3.png)

Unfortunately, it gives even worse results than the GEKKO solution.
