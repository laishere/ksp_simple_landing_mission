from numpy import exp, sqrt
from .vec import *

class ApproachingModel:

    def __init__(self, k: float, b: float, max_acc: float, max_spd: float, accuracy: float=0.5, lock_accuracy_ratio=0.9) -> None:
        self.k = k
        self.b = b
        self.max_acc = max_acc
        self.max_spd = max_spd
        self.accuracy = accuracy
        self.lock_accuracy_ratio = lock_accuracy_ratio

    def spd_at_dist(self, s):
        return sqrt(2 * self.max_acc * s) / (1 + exp(self.k * (s + self.b)))

    def next_acc(self, err, v, dt: float):
        dist = norm(err)
        next_spd = self.spd_at_dist(dist)
        next_spd = min(next_spd, self.max_spd)
        dir = 0. if dist == 0. else err / dist
        next_v = dir * next_spd
        acc = (next_v - v) / dt
        if dist < self.accuracy:
            acc2 = -v / dt # 误差值小于允许精度时希望速度为0
            ratio = 1 - self.lock_accuracy_ratio
            acc = ratio * acc + (1 - ratio) * acc2 # 希望速度尽可能接近0的同时按一定比例减小误差
        return acc

    def find_dist_by_spd(self, spd, accuracy=0.1):
        l, r = 0., 1e8
        while r > l + accuracy:
            m = (r + l) / 2
            v = self.spd_at_dist(m)
            if v > spd: r = m
            else: l = m
        return (l + r) / 2
