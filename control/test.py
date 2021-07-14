from matplotlib import pyplot as plt
from matplotlib import rcParams

from .dynamics import ApproachingModel
from .rocket import Rocket
from .vec import *
from .auto_pilot import AutoPilot

rcParams['font.family'] = 'KaiTi'
rcParams['axes.unicode_minus'] = False
# rcParams['font.sans-serif'] = 'Microsoft YaHei UI'

def plot(run, tf=100, dt=0.1):
    t = 0
    T = []
    A = []
    V = []
    R = []
    while t < tf:
        try: a, v, r = run(t, dt)
        except: break
        t += dt
        T.append(t)
        A.append(a)
        V.append(v)
        R.append(r)
    
    plt.subplot(2, 3, 1)
    plt.title('X轴位置变化')
    plt.plot(T, [r[0] for r in R])
    plt.plot(T, [0. for _ in T], 'r--')

    plt.subplot(2, 3, 2)
    plt.title('X轴速度变化')
    plt.plot(T, [v[0] for v in V])
    plt.plot(T, [0. for _ in T], 'r--')

    plt.subplot(2, 3, 3)
    plt.title('X轴加速度变化')
    plt.plot(T, [a[0] for a in A])
    plt.plot(T, [0. for _ in T], 'r--')

    titles = ['位置', '速度', '加速度']
    data = [R, V, A]
    for i in range(3):
        plt.subplot(2, 3, 4 + i)
        plt.title('水平%s变化'%titles[i])
        plt.plot(T, [d[1] for d in data[i]], label='Y轴')
        plt.plot(T, [d[2] for d in data[i]], label='Z轴')
        plt.plot(T, [0. for _ in T], 'r--')
        plt.legend()

    plt.show()

def landing_test():
    g = 9.81
    r0 = (2400., 10., 10.)
    v0 = (-10., 5., -5.)
    rocket = Rocket(r0, v0, g, radians(45))
    max_up_acc = 2 * g
    max_hor_acc = 5
    model = ApproachingModel(-0.01, -50, max_up_acc * 0.8, 100.)
    # model_yz = ApproachingModel(-0.05, -20, max_hor_acc * 0.5, 50., accuracy=5.)

    def run(t, dt):
        r = rocket.r
        v = rocket.v
        if r[0] < 0.5 and norm(v[0]) < 0.5:
            print('着陆结束于%.2fs，竖直误差：%gm %gm/s，水平误差：%gm %gm/s'%(t, norm(r[0]), norm(v[0]), norm(r[1:3]), norm(v[1:3]))) 
            raise 'done'
        a_x = model.next_acc(-r[0], v[0], 0.2)
        a_x = clamp(a_x, -g, max_up_acc) + g
        a_yz = -r[1:3] * 0.08 - v[1:3] * 0.5
        # a_yz = model_yz.next_acc(-r[1:3], v[1:3], 0.1)
        a_yz = vec_clamp(a_yz, max_hor_acc)
        a_x = max(0.5 * norm(a_yz), a_x)
        a = array((a_x, *a_yz))
        a = vec_clamp_yz(a, radians(60))
        if norm(a) == 0.:
            dir = rocket.dir
        else:
            dir = a / norm(a)
        rocket.thrust(norm(a), dir)
        rocket.update(dt)
        return a, rocket.v.copy(), rocket.r.copy()

    plot(run)

def fly_back_deceleration_test():
    g = 9.81
    r0 = (5000., 200., 100.)
    v0 = (-100., -50., -40.)
    rocket = Rocket(r0, v0, g, radians(45))
    max_hor_acc = g / 0.8
    model_yz = ApproachingModel(-0.03, -40, max_hor_acc * 0.8, 100.)
    global acc_disabled
    acc_disabled = True
    dir0 = -rocket.r[1:3]
    
    def run(t, dt):
        global acc_disabled
        r = rocket.r
        v = rocket.v
        if norm(r[1:3]) < 10. and norm(v[1:3]) < 1.:
            print('减速结束于%.2fs，误差：%gm %gm/s'%(t, norm(r[1:3]), norm(v[1:3])))
            raise 'done'
        a_yz = model_yz.next_acc(-r[1:3], v[1:3], 0.5)
        a_yz = vec_clamp(a_yz, max_hor_acc)
        if acc_disabled and dot(-r[1:3], dir0) < 0.:
            acc_disabled = False
        if acc_disabled and dot(a_yz, -r[1:3]) > 0.:
            a_yz = a_yz * 0. # 禁止加速
        a_x = g * 0.8
        a = array((a_x, *a_yz))
        if norm(a) == 0.:
            dir = rocket.dir
        else:
            dir = a / norm(a)
        rocket.thrust(norm(a), dir)
        rocket.update(dt)
        return a, rocket.v.copy(), rocket.r.copy()
    plot(run)

def auto_pilot_test():
    x = array((0., 1., 0., 0.))
    target = (pi / 2, -1., 1., -1.)
    v = array((2, 3., 3.))
    max_acc = radians(180)
    max_spd = radians(360)
    ap = AutoPilot([max_acc] * 3, max_spd, settling_time=0.1)
    tf = 10
    dt = 0.1
    t = 0.
    T = []
    D = []
    R = []
    A = []
    V = []
    def calc_unit_vectors():
        axis_x = array((1., 0, 0))
        axis_y = array((0., cos(x[0]), sin(x[0])))
        axis_x2 = normalize(x[1:4]) # 当前指向即为当前x轴
        ang_x = vec_ang(axis_x, axis_x2)
        rot_axis = normalize(cross(axis_x, axis_x2))
        axis_y2 = rotate(rot_axis, axis_y, ang_x)
        axis_z2 = cross(axis_x2, axis_y2)
        return [axis_x2, axis_y2, axis_z2]
    while t < tf:
        acc_levels = ap.update(x, target, v)
        real_acc = zeros(3)
        a_levels = []
        v_arr = []
        unit_vectors = calc_unit_vectors()
        axis_x = unit_vectors[0]
        for lv, u in zip(acc_levels, unit_vectors):
            lv = clamp(lv, -1, 1)
            a_levels.append(lv)
            a = lv * max_acc * u
            real_acc += a
            v_arr.append(rad2deg(dot(v, u)))
        dv = real_acc * dt
        d_ang = (v + dv / 2) * dt
        v += dv
        x_d_ang = dot(d_ang, axis_x)
        x[0] += x_d_ang # 
        x[0] = x[0] - (x[0] + pi) // (2 * pi) * 2 * pi # -π 到 π
        yz_d_ang_vec = d_ang - x_d_ang * axis_x
        yz_rot_axis = normalize(yz_d_ang_vec)
        yz_d_ang = dot(yz_d_ang_vec, yz_rot_axis)
        x[1:4] = rotate(yz_rot_axis, x[1:4], yz_d_ang)
        v2 = v * 0.
        unit_vectors2 = calc_unit_vectors()
        for u1, u2 in zip(unit_vectors, unit_vectors2):
            v2 += dot(v, u1) * u2 # 物体旋转时，角速度也会跟着旋转
        v = v2
        T.append(t)
        D.append(rad2deg(vec_ang(x[1:4], target[1:4])))
        R.append(rad2deg(x[0]))
        A.append(a_levels)
        V.append(v_arr)
        t += dt

    plt.subplot(221)
    plt.title('旋转角度变化')
    plt.plot(T, R)

    plt.subplot(222)
    plt.title('方向误差角度变化')
    plt.plot(T, D)

    plt.subplot(223)
    plt.title('xyz角加速度水平变化')
    plt.plot(T, [a[0] for a in A], label='X轴')
    plt.plot(T, [a[1] for a in A], label='Y轴')
    plt.plot(T, [a[2] for a in A], label='Z轴')
    plt.legend()

    plt.subplot(224)
    plt.title('xyz角速度变化')
    plt.plot(T, [v[0] for v in V], label='X轴')
    plt.plot(T, [v[1] for v in V], label='Y轴')
    plt.plot(T, [v[2] for v in V], label='Z轴')
    plt.legend()

    plt.show()

if __name__ == '__main__':
    # landing_test()
    # fly_back_deceleration_test()
    auto_pilot_test()