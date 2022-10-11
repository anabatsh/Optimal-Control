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

* The algorithm (Gauss-Newton) has been implemented entirely. Everything works correctly. 



Current bottlenecks:

* The GN algorithm doesn't work better than the existing method from GEKKO ([Branch and Bound](https://en.wikipedia.org/wiki/Branch_and_bound) based algorithm). 



What I plan to do:

* Find more different examples of the problem and try the method.

---

#### Complicated one: Mixed Integer Non Linear Optimal Control problem:

* There is a time-horizon

<img src="/home/anabatsh/.config/Typora/typora-user-images/image-20221011161606032.png" alt="image-20221011161606032" style="zoom:67%;" />

Current results:

* The first step (S1) of the tree-step GN algorithm has been implemented. 



Current bottlenecks:

* It's hard to extract multidimensional gradients of GEKKO-type functions for the second step (S2) .



What I plan to do:

* Compute each partial derivative independently. But how to do it correctly still doesn't clear enough. 

