import numpy as np
from numpy import array, cross, dot, ndarray, rad2deg, radians, cos, sin, tan, arccos, arcsin, arctan, zeros, pi, sqrt
from numpy.linalg.linalg import norm

def clamp(x, a, b): return max(a, min(b, x))

def vec_clamp(v, max_norm):
    n = norm(v)
    if n == 0.: return v
    return clamp(n, 0, max_norm) * v / n

def vec_ang(a, b):
    na = norm(a)
    nb = norm(b)
    if na == 0. or nb == 0.: return 0.
    cos_ang = np.dot(a, b) / na / nb
    return np.arccos(clamp(cos_ang, -1., 1.)) # 可能存在计算误差使得cos值不在正确范围内

def vec_clamp_yz(vector, ang: float):
    '''
    vector: 需要限制指向的向量
    ang:    离yz平面最小角度，单位是弧度
    '''
    vector = vector.copy()
    x = vector[0]
    yz = vector[1:3] # yz分量
    n_yz = norm(yz)
    max_n = abs(x / np.tan(ang))
    if max_n < n_yz:
        yz = yz / n_yz * max_n
    vector[1:3] = yz
    return vector

def vec_around(a, b, ang) -> ndarray:
    target_ang = vec_ang(a, b)
    rot_ang = clamp(target_ang, -ang, ang)
    rot_axis = normalize(cross(a, b))
    return rotate(rot_axis, a, rot_ang)

def rotate(k, v, ang):
    return np.cos(ang) * v + (1 - np.cos(ang)) * np.dot(v, k) * k + np.sin(ang) * cross(k, v)

def normalize(v):
    n = norm(v)
    if n == 0.: return v
    return v / n