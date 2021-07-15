import sys
from threading import Thread, Event
import krpc
from ksp import Rocket, surface_ref, find_vessel_by_name, surface_position
from time import sleep, time
from control.vec import *
from control.dynamics import ApproachingModel
from queue import Queue

start_time = time()
last_debug_new_line_time = 0
def debug(*args):
    global last_debug_new_line_time
    msg = ' '.join([str(e) for e in args])
    dt = time() - start_time
    out = '%.2fs: %s'%(dt, msg)
    whitespace = ''.join([' ' for _ in range(80 - len(msg))])
    if time() - last_debug_new_line_time > 1:
        last_debug_new_line_time = time()
        out = '\n' + out
    else:
        print('\r', end='')
    print(out, end=whitespace)
    sys.stdout.flush()

def launch(rocket: Rocket, after_booster_seperation=None, after_first_stage_seperation=None):
    '''
    发射：先等待火箭速度大于50，然后开始重力转弯，为了保证每次运行分离时，位置和速度基本一致，
    我们一开始不直接跟随速度方向，而是先根据时间均匀减小俯仰角度，等到速度大小达到一定值时，再跟随速度俯仰
    '''
    sleep(5)
    rocket.control.activate_next_stage() # 点火
    rocket.control.throttle = 1. # 节流阀开到最大
    rocket.ap.dir_model.lock_accuracy_ratio = 0.5 # 设置姿态控制的方向运动模型刹车力度
    rocket.ap.update_config(settling_time=3.) # 设置大一点的姿态控制校正时间，以免姿态调整过于激烈
    sleep(0.5)
    rocket.control.activate_next_stage() # 发射塔分离
    flight = rocket.flight()
    while is_alive() and rocket.velocity()[0] < 20.: # 等待速度大于20m/s
        rocket.update_ap((1, 0, 0))
        sleep(0.1)
    target_roll = radians(0) # 目标旋转角度
    while is_alive() and rocket.velocity()[0] < 50.: # 等待速度达到50m/s 
        rocket.update_ap((1, 0, 0), target_roll)
        sleep(0.1)
    print('开始重力转弯')
    orbit = rocket.orbit
    up = array((1., 0, 0))
    pitch_spd = 0.45
    pitch0 = flight.pitch
    lock_pitch = None
    max_angle_of_attack = 5
    t = rocket.met
    state = 1
    main_engin_cutoff_time = 0.
    while is_alive() and orbit.apoapsis_altitude < 200000: # 循环执行指令直到高点海拔达到200km
        v = rocket.velocity()
        spd = norm(v)
        vel_ang = pi / 2 - vec_ang(up, v) # 计算速度俯仰
        vel_ang = rad2deg(vel_ang) # 弧度转换成角度制
        dt = rocket.met - t
        pitch = pitch0 - pitch_spd * dt
        if lock_pitch is None and spd > 800: # 直到速度大小大于800m/s，设置最后一次俯仰
            lock_pitch = pitch
        if lock_pitch is not None:
            pitch = min(vel_ang, lock_pitch) # 取速度俯仰和跟随速度俯仰前的最后一次俯仰的最小值作为当前俯仰
            # 即，我们希望此时跟随速度俯仰，但是如果速度俯仰比较大，我们暂时不跟随
        if state == 1 and orbit.apoapsis_altitude > 60000: # 助推器分离
            state += 1
            rocket.control.activate_next_stage() # 分离
            if after_booster_seperation is not None:
                after_booster_seperation() # 调用回调函数
        if state == 2 and orbit.apoapsis_altitude > 120000: # 一级关机
            state += 1
            rocket.thrust(0.) # 节流阀设为0
            main_engin_cutoff_time = rocket.met # 记录一级关机时间
        if state > 2:
            t = rocket.met - main_engin_cutoff_time
            if state == 3 and t > 0.5:
                rocket.control.activate_next_stage() # 分离
                state += 1
            elif state == 4 and t > 1.:
                rocket.control.activate_next_stage() # 二级点火
                rocket.control.throttle = 0.16 # 以较小的节流阀开度脱离一级，以免温度过高炸掉一级
                state += 1
            elif state == 5 and t > 4.:
                rocket.control.throttle = 1 # 加大油门
                if after_first_stage_seperation is not None:
                    after_first_stage_seperation() # 调用回调函数
                state += 1                
                
        pitch = clamp(pitch, vel_ang - max_angle_of_attack, vel_ang + max_angle_of_attack)
        pitch = radians(pitch)
        rocket.update_ap((sin(pitch), 0, cos(pitch)), target_roll)
        sleep(0.1)
    rocket.control.throttle = 0.

def is_alive():
    if mission_abort is None: return True
    return not mission_abort.is_set()

def booster_back(rocket: Rocket, myself_ready: Event, another_ready: Event, roll):
    rocket.control.rcs = True # 开始RCS
    spd_hor = 250
    spd_ver = 0
    max_acc = 40
    g0 = rocket.orbit.body.surface_gravity
    g = array((-g0 * 0.9, 0, 0))
    def calc_dv(v0=None): # 计算dv，即：目标速度 - 当前速度
        r = rocket.position()
        v = rocket.velocity()
        dir = - r[1:3] # 目标方向
        dir = normalize(dir) # 取单位向量
        vel_hor = spd_hor * dir # 水平速度为：大小 * 目标方向的单位向量
        target_v0 = clamp(v[0], spd_ver, spd_ver + 5) # 允许竖直速度存在误差
        vt = array([target_v0, *vel_hor]) # 目标速度
        if v0 is not None:
            v[0] = v0 # 调整方向需要一定时间，v0参数可以提前预测点火开始前的竖直速度，可以使得点火时方向更精准
        return vt - v
    adjust_dir_time = 15.
    v0 = rocket.velocity()[0] - g0 * adjust_dir_time
    dir = calc_max_thruster_acc_at_dir(rocket, g, calc_dv(v0), max_acc) # dv就是目标加速度方向，我们计算实际推力加速度方向作为目标方向
    while is_alive(): # 此循环是调整点火方向
        cur_dir = rocket.direction(rocket.ref)
        cur_dir = array(cur_dir) # 当前方向
        ang = vec_ang(dir, cur_dir) # 方向角度误差
        if ang < radians(1): 
            myself_ready.set() # 如果方向角度误差 < 1° 则 设置事件变量，告诉另一个助推器，自己已经准备好点火
        if myself_ready.is_set() and another_ready.is_set(): # 等待自己和另一个助推器准备好就跳出循环
            break
        rocket.update_ap(dir, roll) # 更新方向
        sleep(0.1)
    min_dv = 1e9
    while is_alive(): # 点火调整速度
        dv = calc_dv() # 计算dv
        cur_dv = norm(dv)
        min_dv = min(min_dv, cur_dv)
        if cur_dv < 20 and cur_dv > min_dv: break # 如果当前dv < 20 并且 dv开始增大了，也就是刚刚调整过头，我们就跳出循环，完成返航点火任务
        acc = calc_max_thruster_acc_at_dir(rocket, g, dv, max_acc) # 计算实际推力加速度
        rocket.update_ap(acc) # 调整方向
        rocket.thrust(norm(acc)) # 根据加速度大小设置节流阀开度
        sleep(0.05)
    rocket.thrust(0.) # 不要忘了熄火

def calc_max_thruster_acc_at_dir(rocket, g, dir, max_acc):
    a = min(rocket.available_thrust / rocket.mass, max_acc)
    b = norm(g)
    A = vec_ang(dir, g)
    sinB = b * sin(A) / a # 正弦定理
    B = arcsin(sinB) # B必定为锐角
    C = pi - A - B
    acc_norm = a * sin(C) / sin(A) # 正弦定理
    acc_norm = min(max_acc, acc_norm)
    acc = normalize(dir) * acc_norm
    return acc - g # 实际加速度

def entry_burn(rocket: Rocket):
    '''
    再入点火和助推器返航点火差不多，未标明的注释可以参照上面的
    '''
    rocket.control.rcs = True
    while is_alive() and rocket.velocity()[0] > 0:
        rocket.update_ap((1, 0, 0))
        sleep(0.05)
    g0 = rocket.orbit.body.surface_gravity
    g = array((-g0 * 0.85, 0, 0))
    v = rocket.velocity()
    r = rocket.position()
    entry_burn_alt = 50000.
    g1 = 0.79 * g0
    entry_burn_spd_ver = -sqrt(2 * g1 * norm(r[0] - entry_burn_alt) + v[0] ** 2) # 计算点火前的竖直速度
    spd_ver = -300
    spd_hor = 160
    max_acc = 40
    def calc_dv(v0=None):
        v = rocket.velocity()
        dir = -rocket.position()[1:3]
        dir = normalize(dir)
        vel_hor = spd_hor * dir
        target_v0 = clamp(v[0], spd_ver, spd_ver + 5) # 允许竖直方向存在一定误差
        vt = array((target_v0, *vel_hor))
        if v0 is not None:
            v[0] = v0
        return vt - v
    while is_alive() and rocket.position()[0] > entry_burn_alt:
        acc = calc_max_thruster_acc_at_dir(rocket, g, calc_dv(entry_burn_spd_ver), max_acc) # 根据我们计算的点火前速度计算我们的预测推力方向，提前调整好方向
        rocket.update_ap(acc)
        sleep(0.05)
    min_err = 1e9
    last_dir = calc_max_thruster_acc_at_dir(rocket, g, calc_dv(), max_acc)
    print('entry burn开始于高度：', rocket.position()[0], '速度：', rocket.velocity())
    while is_alive():
        dv = calc_dv()
        err = norm(dv)
        min_err = min(min_err ,err)
        if err < 20 and err >= min_err: break
        a = calc_max_thruster_acc_at_dir(rocket, g, dv, max_acc)
        if vec_ang(last_dir, a) < radians(0.5): 
            last_dir = a # 如果方向变化较小则允许改变，否则不改变方向
        rocket.update_ap(last_dir)
        rocket.thrust(norm(a))
    rocket.release_ap_control()
    rocket.thrust(0.)
    dir = -rocket.position()[1:3]
    err_ang = rad2deg(vec_ang(dir, rocket.velocity()[1:3]))
    print('再入制动完成, 速度误差：%s, 方向误差: %g度'%(str(calc_dv()), err_ang))

def landing_burn1(rocket: Rocket, roll=None):
    rocket.control.brakes = True # 打开栅格翼，因为大气开始稠密起来了，我们可以利用栅格翼辅助姿态控制
    rocket.ap.dir_model.lock_accuracy_ratio = 0.75
    max_hor_acc = 30
    model = ApproachingModel(-0.01, -50, max_hor_acc * 0.5, 500, accuracy=100) # 水平方向的运动模型
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        v_hor = v[1:3]
        err_hor = -r[1:3]
        dist = norm(err_hor)
        min_dist = model.find_dist_by_spd(norm(v_hor)) + 6. * norm(v_hor) # 计算减速开始前6秒的水平距离
        if dist <= min_dist: break # 如果即将开始减速，则跳出循环准备下一步的减速
        dir_hor = normalize(err_hor)
        v_hor_err = v[1:3] - dot(v[1:3], dir_hor) * dir_hor
        dir = -v + array((0., *v_hor_err)) * 5 # 跟随速度方向，并且按照一定比例消除水平方向误差
        rocket.update_ap(dir, roll)
        err_ang = rad2deg(vec_ang(v[1:3], dir_hor))
        debug('方向误差：%g度，水平速度误差：%s，%g %g'%(err_ang, str(v_hor_err), dist, min_dist))
    start = False
    dir = -rocket.velocity() # 取当前速度方向作为减速方向
    dir2 = dir.copy()
    dir2[0] = 0. # dir2为平躺方向
    dir = vec_around(dir, dir2, radians(10)) # 我们希望dir往平躺方向靠近，这样可以增大水平加速度分量，但是dir朝着dir2靠近不允许超过10°，目的是限制迎角
    dir = vec_around(array([1, 0, 0]), dir, radians(30)) # 又希望dir离竖直方向不要超过30°
    min_spd = 1e9
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        err_hor = -r[1:3]
        dist = norm(err_hor)
        spd = norm(v[1:3])
        min_spd = min(min_spd - 0.05, spd)
        if spd < 5 or (spd < 20 and spd > min_spd):
            break # 如果速度大小 < 5m/s 或者 速度大小小于20m/s 并且速度开始增大了，也就是出现了控制误差比较大的情况，我们就跳出循环
        dir2 = dir
        acc_hor2 = 0.
        if not start: # 如果刹车点火还没启动
            min_dist = model.find_dist_by_spd(norm(v[1:3])) + 1. * norm(v_hor)
            start = dist <= min_dist # 检查是否应该启动了，如果水平距离 <= min_dist，说明大概还有1秒就要开始点火
            acc = 0.
            if start:
                print('\nlanding burn1开始于高度：', r[0], '速度：', v)
        else:
            acc_hor = model.next_acc(err_hor, v[1:3], 0.5) # 利用运动模型计算水平加速度
            acc_hor = vec_clamp(acc_hor, max_hor_acc) # 限制加速度大小
            dir_hor = normalize(dir[1:3]) # 当前火箭方向的水平分量的单位向量
            acc_hor_proj = dot(acc_hor, dir_hor) # 计算加速度在水平方向上的投影大小
            acc1 = max(0., acc_hor_proj) * norm(dir) / norm(dir[1:3]) # 为了满足水平加速度大小，计算当前朝向的加速度大小
            dir2 = normalize(dir) * acc1 # 当前加速度方向，也就是dir的方向，还包括了加速度大小acc1
            if acc1 > 0.:
                acc_hor2 = (acc_hor - acc_hor_proj * dir_hor) * 0.3 # 水平分量垂直于dir的分量，我们需要考虑不与dir平行的分量，否则无法消除这个方向上的误差
                acc_hor2 = vec_clamp(acc_hor2, acc1 * 0.1) # 限制垂直于dir的水平分量大小
                dir2 += array((0., *acc_hor2)) # 加上这个水平分量
            acc = norm(dir2) # 计算最后的加速度大小
        debug('水平误差：%s，速度大小：%g，加速度： %s %s'%(str(err_hor), norm(v[1:3]), str(acc), str(acc_hor2)))
        rocket.update_ap(dir2) # 更新加速方向
        rocket.thrust(acc) # 更新节流阀
    rocket.thrust(0.)
    rocket.release_ap_control()

def landing_burn2(rocket: Rocket, rocket_height):
    rocket.ap.dir_model.lock_accuracy_ratio = 0.75
    g = rocket.orbit.body.surface_gravity
    max_ver_acc = min(30, rocket.available_thrust / rocket.mass - g)
    model = ApproachingModel(-0.01, -50, max_ver_acc * 0.8, 500, accuracy=3, lock_accuracy_ratio=0.1) # 竖直方向上的运动模型
    final_landing_started = False
    rocket.ap.update_config(settling_time=0.5)
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        err_ver = rocket_height - r[0]
        if abs(err_ver) < 10 and norm(v[0]) < 1.5: break # 如果竖直误差大小 < 10m，而且竖直速度 < 1.5m/s，那么着陆完成，跳出循环
        if abs(err_ver) < 100:
            rocket.control.gear = True # 还有100m时开启着陆支架
        acc_ver = model.next_acc(err_ver, v[0], 0.3) # 利用运动模型计算竖直加速度
        acc_ver = clamp(acc_ver, -g, max_ver_acc) + g # 限制加速度大小，因为计算的加速度时合加速度大小，所以需要 - (-g)，即 + g
        acc_hor = -r[1:3] * 0.1 - v[1:3] * 0.5 # 水平方向上使用比例控制
        dir = (1., 0, 0)
        acc = 0.
        if r[0] < 3000: # 如果高度小于3000m，则开始考虑着陆点火
            if norm(acc_hor) < 3 and acc_ver == 0.:
                acc_hor *= 0. # 如果存在水平误差，而为了校正水平误差需要的水平加速度比较小，我们直接忽略，令水平加速度为0
            acc_ver = max(acc_ver, norm(acc_hor) * 1.5) # 竖直加速度大小不小于水平加速度大小的1.5倍
            acc = array(((acc_ver, *acc_hor))) # 合并加速度
            acc = vec_clamp_yz(acc, radians(80)) # 限制加速度方向的俯仰 >= 80 °
            dir = acc
            if norm(acc) == 0.:
                dir = (1., 0, 0) # 如果没有加速度，则保持竖直向上
            debug(acc, r, v)
            acc = norm(acc) # 计算加速度大小
            if not final_landing_started:
                final_landing_started = acc > 0. # 更新着陆点火启动的标志量
        if not final_landing_started: # 如果还没开始点火，那么采用空气动力滑行
            max_ang = 15
            min_ang = 3
            max_ang_height = 10000
            min_ang_height = 3000
            # 高海拔允许大的迎角，低海拔则设置小迎角限制
            ang = (r[0] - min_ang_height) / (max_ang_height - min_ang_height) * (max_ang - min_ang) + min_ang
            ang = clamp(ang, min_ang, max_ang) # 限制角度范围
            dir_hor = (r[1:3] * 0.08 + v[1:3] * 0.5) * 0.5 # 比例控制计算火箭倾斜方向
            dir = array([max(norm(dir_hor) * 3, 10), *dir_hor]) # 考虑竖直分量大小，合并方向向量
            dir = vec_around(-v, dir, radians(ang)) # 根据ang限制迎角
            acc = 0.
            debug(r[1:3], v[1:3])
        rocket.update_ap(dir) # 更新方向
        rocket.thrust(acc) # 更新节流阀
    rocket.release_ap_control()
    rocket.thrust(0.)

def launch_and_booster_back_mission():
    def mission():
        vessel = space_center.active_vessel
        launch(Rocket(space_center, vessel), after_booster_seperation=booster_landing)
    
    task = Thread(target=mission) # 构造任务线程
    thread_queue.put(task) # 加入任务队列
    task.start() # 启动线程

def booster_landing():
    booster_left = find_vessel_by_name(space_center, 'BL')
    booster_right = find_vessel_by_name(space_center, 'BR')
    landing_site_left = (-0.185401781224271, -74.4729352356523)
    landing_site_right = (-0.205703802260084, -74.4730931260344)
    is_bl_ready = Event()
    is_br_ready = Event()

    def mission(vessel, site, myself_ready, another_ready, roll):
        ref = surface_ref(vessel.orbit.body, *site)
        rocket = Rocket(space_center, vessel, ref)
        booster_back(rocket, myself_ready, another_ready, roll) # 助推器返回
        landing_burn1(rocket, roll) # 水平刹车
        landing_burn2(rocket, 16) # 滑行和着陆
        print('\n助推器着陆任务结束')
    
    args = [
        (booster_left, landing_site_left, is_bl_ready, is_br_ready, radians(-90)), 
        (booster_right, landing_site_right, is_br_ready, is_bl_ready, radians(90))
    ]

    for a in args:
        task = Thread(target=mission, args=a)
        thread_queue.put(task)
        task.start()

def first_stage_landing():
    def mission():
        first_stage = find_vessel_by_name(space_center, 'S1')
        target = (-0.0933756132563131, -56.8000001513979)
        ref = surface_ref(first_stage.orbit.body, *target)
        rocket = Rocket(space_center, first_stage, ref)
        entry_burn(rocket) # 再入点火
        landing_burn1(rocket) # 水平刹车
        landing_burn2(rocket, 18.85) # 滑行和着陆
        print('\n一级火箭着陆任务结束')
    
    task = Thread(target=mission)
    thread_queue.put(task)
    task.start()

def launch_and_first_stage_landing_mission():
    def mission():
        vessel = space_center.active_vessel
        launch(Rocket(space_center, vessel), after_first_stage_seperation=first_stage_landing)

    task = Thread(target=mission)
    thread_queue.put(task)
    task.start()

thread_queue = Queue()
mission_abort = Event()

if __name__ == '__main__':
    conn = krpc.connect('Mission II')
    space_center = conn.space_center

    launch_and_booster_back_mission() # 发射和助推器回收
    # launch_and_first_stage_landing_mission() # 发射和一级火箭回收

    # 等待所有线程结束
    while True:
        try:
            if thread_queue.empty(): break
            task = thread_queue.get_nowait()
            task.join(0.5)
            if task.is_alive():
                thread_queue.put(task)
        except KeyboardInterrupt as e:
            # 键盘ctrl+c打断，发起停止任务事件
            mission_abort.set()