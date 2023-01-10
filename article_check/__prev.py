# N = 5
# d = 3

# a_1d = np.linspace(-5, 5, N)
# a_nd = np.array([a_1d]*d).T
# x_nd = a_nd + 1.5 #np.random.rand(N, 1) * 2
# x_1d = x_nd[:, 0]
# xand = list(zip(x_nd, a_nd))

# f_1 = lambda x, u, i: np.array([x, u, i])
# f_0 = lambda x, u, i: 0.5 * sum(f_1(x, u, i) ** 2)
# f_l = lambda x, x0: linear(f_0, x, x0)
# f_g = lambda x, x0: gauss_newton(f_0, f_1, x, x0)

# ya_0 = [f_0(*a) for x, a in xand]
# yx_0 = [f_0(*x) for x, a in xand]
# y_l = [f_l(x, a) for x, a in xand]
# y_g = [f_g(x, a) for x, a in xand]

# xa = np.hstack([x_1d, a_1d])
# yxa = np.array(yx_0 + ya_0)
# idx = np.argsort(xa)
# xa, yxa = xa[idx], yxa[idx]

# plt.figure(figsize=(15, 5))
# plt.plot(xa, yxa, '--', c='black', alpha=0.7, label='f')

# plt.scatter(a_1d, ya_0, c='red', label='known points')
# plt.scatter(x_1d, yx_0, c='black', label='true points')
# plt.scatter(x_1d, y_l, c='green', label='linear approx')
# plt.scatter(x_1d, y_g, c='blue', label='gauss-newton approx')

# _, _, ymin, ymax = plt.axis()
# plt.ylim(ymin, ymax+0.1)
# plt.xticks(x_1d)

# for n in range(N):
#     plt.plot([a_1d[n], x_1d[n]], [ya_0[n], y_l[n]], c='black')
#     plt.vlines(x_1d[n], ymin, yx_0[n], linestyles='--', colors='black', alpha=0.7)

# plt.legend()
# plt.show()


# --------------------------------------------------------------------------------------

# def linear(f, x, x0):
#     f_grad = optimize.approx_fprime(x0, f, 1e-6)
#     delta = x - x0
#     return f(x0) + np.dot(f_grad, delta)

# def quadratic(f, x, x0, B):
#     f_linear = linear(f, x, x0)
#     delta = x - x0
#     return f_linear + 0.5 * delta @ B @ delta

# def b_gn(f, x, x0):
#     f_grad = optimize.approx_fprime(x0, f, 1e-6)
#     return f_grad @ f_grad.T

# def gauss_newton(f, f1, x, x0):
#     B_GN = b_gn(f1, x, x0)
#     return quadratic(f, x, x0, B_GN)

# d = 5
# x0 = np.array([2.0]*d)
# x_ = np.linspace(-5, 5, 11)
# x = np.array([x_]*d).T
# B = np.eye(d)

# f_1 = lambda x: x
# f_0 = lambda x: 0.5 * sum(f_1(x) ** 2)
# f_l = lambda x, x0: linear(f_0, x, x0)
# f_g = lambda x, x0: gauss_newton(f_0, f_1, x, x0)

# y_0 = [f_0(x_i) for x_i in x]
# y_l = [f_l(x_i, x0) for x_i in x]
# y_g = [f_g(x_i, x0) for x_i in x]
    
# plt.plot(x_, y_0, label='original')
# plt.plot(x_, y_l, label='linear')
# plt.plot(x_, y_g, label='Gauss-Newton')
# plt.xticks(x_)
# _, _, ymin, ymax = plt.axis()
# plt.vlines(x0[0], ymin, f_0(x0), colors='black')
# plt.legend()
# plt.show()