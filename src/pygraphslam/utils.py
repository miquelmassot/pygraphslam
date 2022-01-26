import numpy as np


def hessian_matrix(hessian_fun):
    hessian = np.ndarray((3, 3))
    for i in range(3):
        for j in range(3):
            hessian[i, j] = hessian_fun(i, j)
    return hessian


def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:, order]
