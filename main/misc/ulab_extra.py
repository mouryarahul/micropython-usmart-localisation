import math
import random
import ulab

__all__ = ['gauss', 'uniform', 'randn', 'rand', 'l2norm', 'median']


TWOPI = 2 * math.pi

def gauss(mu: float = 0., sigma: float = 1.) -> float:
    return sigma * (math.sqrt(-2.0 * math.log(random.random())) * math.cos(TWOPI * random.random())) + mu


def uniform(lb: float = 0., ub: float = 1.) -> float:
    return random.uniform(lb, ub)


def randn(shape: tuple, mu: float = 0., sigma: float = 1.) -> ulab.array:
    return ulab.array([[gauss(mu, sigma) for j in range(shape[1])] for i in range(shape[0])], dtype=ulab.float)


def rand(shape: tuple, lb: float = 0., ub: float = 1.) -> ulab.array:
    return ulab.array([[uniform(lb, ub) for j in range(shape[1])] for i in range(shape[0])], dtype=ulab.float)


def l2norm(a: ulab.array) -> float:
    assert isinstance(a, ulab.array) and (a.shape()[0] == 1 or a.shape()[1] == 1), "Input should be column or row array!"
    return ulab.vector.sqrt(ulab.numerical.sum(a ** 2))

def diag(a: ulab.array) -> ulab.array:
    assert isinstance(a, ulab.array), "Input should be of type ulab.array!"
    m, n = a.shape()
    itemsize = a.itemsize()
    if itemsize == 1:
        dtype = ulab.int8
    elif itemsize == 2:
        dtype = ulab.int16
    elif itemsize == 8:
        dtype = ulab.float
    else:
        raise Exception("Unknown datatype of the input array!")

    if m <= 1 and n <= 1:
        return a
    elif (m == 1 and n > 1) or (m > 1 and n == 1):
        r = m if m > n else n
        b = ulab.eye(r, dtype=dtype)
        for i in range(r):
            b[i, i] = a[i]
        return b
    elif m == n:
        b = ulab.zeros(m, dtype=dtype)
        for i in range(m):
            b[i] = a[i][i]
        return b
    else:
        raise Exception("Input should be square matrix!")


"""
def norm(a: ulab.array, ord=None, axis=None) -> float:
    assert isinstance(a, ulab.array) and a.shape()[0] > 0 and a.shape()[1] > 0, "Input should be non-empty ulab.array!"
    if a.shape()[0] == 1 or a.shape()[1] == 1:
        if ord is None:
            return l2norm(a)
        elif ord == 'fro' or ord == 'nuc':
            raise Exception("Frobenius and Nuclear norm are not defined for row or column vector!")
        elif ord == float('Inf'):
            return ulab.numerical.max(abs(a))[0]
        elif ord == -float('Inf'):
            return ulab.numerical.min(abs(a))[0]
        elif ord == 0:
            return ulab.numerical.sum([a != 0.])
        elif ord == 1:
            return ulab.numerical.sum(abs(a))
        elif ord == -1:
            return (ulab.numerical.sum(abs(a) ** -1)) ** -1
        elif ord == 2:
            return l2norm(a)
        elif ord == -2:
            return (ulab.numerical.sum(abs(a) ** -2)) ** -0.5
        else:
            raise Exception("Not supported ord !")
    else:
        if ord is None or ord == 'fro':
            return ulab.vector.sqrt(ulab.numerical.sum(a ** 2))
        elif ord == 'nuc':
            raise Exception("Not implemented!")
        elif ord == float('Inf'):
            return ulab.numerical.max(ulab.numerical.sum(abs(a), axis=1))
        elif ord == -float('Inf'):
            return ulab.numerical.min(ulab.numerical.sum(abs(a), axis=1))
        elif ord == 0:
            raise Exception("Not defined for matrix!")
        elif ord == 1:
            return ulab.numerical.max(ulab.numerical.sum(abs(a), axis=0))
        elif ord == -1:
            return ulab.numerical.min(ulab.numerical.sum(abs(a), axis=0))
        elif ord == 2:
            raise Exception("Not implemented!")
        elif ord == -2:
            raise Exception("Not implemented!")
        else:
            raise Exception("Not supported ord !")
"""

def median(a: ulab.array, axis=None):
    assert isinstance(a, ulab.array), "Input should be ulab.array!"
    m, n = a.shape()
    a = ulab.numerical.sort(a, axis=axis)
    if axis is None:
        s = m * n
        if (s % 2) == 0:  # even
            l = math.floor((s + 1) / 2)
            u = math.ceil((s + 1) / 2)
            result = (a[l-1] + a[u-1]) / 2
        else:
            l = math.floor((s + 1)/2)
            result = a[l-1]
        return result
    elif axis == 0:
        result = ulab.zeros(n, dtype=ulab.float)
        if m % 2 == 0:
            l = math.floor((m + 1) / 2)
            u = math.ceil((m + 1) / 2)
            for i in range(n):
                result[i] = (a[l-1, i] + a[u-1, i]) / 2
        else:
            l = math.floor((m + 1) / 2)
            for i in range(n):
                result[i] = a[l-1, i]
        return result
    elif axis == 1:
        result = ulab.zeros(m, dtype=ulab.float)
        if n % 2 == 0:
            l = math.floor((n + 1) / 2)
            u = math.ceil((n + 1) / 2)
            for i in range(m):
                result[i] = (a[i, l-1] + a[i, u-1]) / 2
        else:
            l = math.floor((n + 1) / 2)
            for i in range(m):
                result[i] = a[i, l-1]
        return result
    else:
        raise Exception("Allowed axis are None, 0, and 1!")

