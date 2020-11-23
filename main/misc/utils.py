import math
import random

__all__ = ['distance', 'num_combs', 'num_perms', 'random_combinations']

def distance(point1: list, point2: list) -> float:
    assert len(point1) == len(point2), "Two inputs should be lists of float numbers and equal length!"
    dist = 0.0
    for p, q in zip(point1, point2):
        dist += (p - q)**2
    return math.sqrt(dist)

def num_combs(n, r):
    if r > n:
        return 0
    else:
        return math.factorial(n)/(math.factorial(r)*math.factorial(n-r))

def num_perms(n, r):
    if r > n:
        return 0
    else:
        return math.factorial(n)/math.factorial(n-r)

def random_combinations(iterables, r, n):
    max_num_combs = num_combs(len(iterables), r)
    num_subsets = min(max_num_combs, n)
    subsets = set()
    while len(subsets) < num_subsets:
        item = random.choice(iterables)
        subset = {item}
        while len(subset) < 3:
            subset.add(random.choice(iterables))
        subsets.add(frozenset(subset))
    return subsets

