from .vec import *

class SimpleRocket:

    def __init__(self, m: float, r0, v0) -> None:
        '''
        m:  质量
        r0: 初始位置
        v0: 末位置
        '''
        self.m = m
        self.r = r0
        self.v = v0
        if type(self.r) is tuple:
            self.r = array(self.r, dtype=float)
            self.v = array(self.v, dtype=float)

    def apply_force(self, f, dt: float) -> None:
        '''
        f:  合力
        dt: 作用时间
        '''
        if type(f) is tuple:
            f = array(f, dtype=float)
        a = f / self.m
        dv = a * dt
        self.r += self.v * dt + dv * dt  / 2
        self.v += dv        

class Rocket:

    def __init__(self, r0, v0, g, dir_spd) -> None:
        self.r = array(r0, dtype=float)
        self.v = array(v0, dtype=float)
        self.g = array((-g, 0, 0))
        self.dir_spd = dir_spd
        self.roll_ = 0.
        self.dir = array((1., 0., 0.))
    
    def thrust(self, acc: float, dir):
        self.acc = acc
        self.acc_dir = array(dir)
    
    def update(self, dt) -> None:
        ang = vec_ang(self.dir, self.acc_dir)
        max_d_ang = self.dir_spd * dt
        ang = clamp(ang, -max_d_ang, max_d_ang)
        rot = cross(self.dir, self.acc_dir)
        self.dir = rotate(normalize(rot), self.dir, ang)
        real_acc = self.dir * self.acc + self.g
        dv = real_acc * dt
        self.r += (self.v + dv / 2) * dt
        self.v += dv

def simple_rocket_test():
    m = 10
    a = 3
    tc = m * a
    dt = 3
    v0 = 10
    rocket = SimpleRocket(m, (0, 0, 0), (v0, 0, 0))
    rocket.apply_force((tc, 0, 0), dt)
    assert np.array_equal(rocket.r, (0.5 * a * dt ** 2 + v0 * dt, 0, 0))
    assert np.array_equal(rocket.v, (v0 + a * dt, 0, 0))
    print('简单火箭运动计算测试...通过')

def rocket_test():
    v0 = 10.
    a = 10.
    t = 5
    rocket = Rocket((0., 0, 0), (v0, 0, 0), 0, 1)
    rocket.thrust(a, (1, 0, 0))
    rocket.update(t / 2)
    rocket.update(t / 2)
    assert rocket.r[0] == v0 * t + 0.5 * a * t ** 2
    assert rocket.v[0] == v0 + a * t
    
    print('火箭运动计算测试...通过')

if __name__ == '__main__':
    simple_rocket_test()
    rocket_test()