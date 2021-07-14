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
    sleep(5)
    rocket.control.activate_next_stage()
    rocket.control.throttle = 1.
    rocket.ap.dir_model.lock_accuracy_ratio = 0.5
    rocket.ap.update_config(settling_time=3.)
    sleep(0.5)
    rocket.control.activate_next_stage()
    flight = rocket.flight()
    while is_alive() and rocket.velocity()[0] < 20.:
        rocket.update_ap((1, 0, 0))
        sleep(0.1)
    target_roll = radians(0)
    while is_alive() and rocket.velocity()[0] < 50.: 
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
    while is_alive() and orbit.apoapsis_altitude < 200000:
        v = rocket.velocity()
        spd = norm(v)
        vel_ang = pi / 2 - vec_ang(up, v)
        vel_ang = rad2deg(vel_ang)
        dt = rocket.met - t
        pitch = pitch0 - pitch_spd * dt
        if lock_pitch is None and spd > 800:
            lock_pitch = pitch
        if lock_pitch is not None:
            pitch = min(vel_ang, lock_pitch)
        if state == 1 and orbit.apoapsis_altitude > 60000: # 助推器分离
            state += 1
            rocket.control.activate_next_stage()
            if after_booster_seperation is not None:
                after_booster_seperation()
        if state == 2 and orbit.apoapsis_altitude > 120000: # 一级关机
            state += 1
            rocket.thrust(0.)
            main_engin_cutoff_time = rocket.met
        if state > 2:
            t = rocket.met - main_engin_cutoff_time
            if state == 3 and t > 0.5:
                rocket.control.activate_next_stage() # 分离
                state += 1
            elif state == 4 and t > 1.:
                rocket.control.activate_next_stage() # 二级点火
                rocket.control.throttle = 0.16
                state += 1
            elif state == 5 and t > 3.:
                rocket.control.throttle = 1 # 加大油门
                if after_first_stage_seperation is not None:
                    after_first_stage_seperation()
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
    rocket.control.rcs = True
    spd_hor = 250
    spd_ver = 0
    max_acc = 40
    g0 = rocket.orbit.body.surface_gravity
    g = array((-g0 * 0.9, 0, 0))
    def calc_dv(v0=None):
        r = rocket.position()
        v = rocket.velocity()
        dir = - r[1:3]
        dir = normalize(dir)
        vel_hor = spd_hor * dir
        target_v0 = clamp(v[0], spd_ver, spd_ver + 5) # 允许竖直速度存在误差
        vt = array([target_v0, *vel_hor])
        if v0 is not None:
            v[0] = v0
        return vt - v
    adjust_dir_time = 15.
    v0 = rocket.velocity()[0] - g0 * adjust_dir_time
    dir = calc_max_thruster_acc_at_dir(rocket, g, calc_dv(v0), max_acc)
    while is_alive():
        cur_dir = rocket.direction(rocket.ref)
        cur_dir = array(cur_dir)
        ang = vec_ang(dir, cur_dir)
        if ang < radians(1): 
            myself_ready.set()
        if myself_ready.is_set() and another_ready.is_set():
            break
        rocket.update_ap(dir, roll)
        sleep(0.1)
    min_dv = 1e9
    while is_alive():
        dv = calc_dv()
        cur_dv = norm(dv)
        min_dv = min(min_dv, cur_dv)
        if cur_dv < 20 and cur_dv > min_dv: break
        acc = calc_max_thruster_acc_at_dir(rocket, g, dv, max_acc)
        rocket.update_ap(acc)
        rocket.thrust(norm(acc))
        sleep(0.05)
    rocket.thrust(0.)

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
    entry_burn_spd_ver = -sqrt(2 * g1 * norm(r[0] - entry_burn_alt) + v[0] ** 2)
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
        acc = calc_max_thruster_acc_at_dir(rocket, g, calc_dv(entry_burn_spd_ver), max_acc)
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
    rocket.control.brakes = True
    rocket.ap.dir_model.lock_accuracy_ratio = 0.75
    max_hor_acc = 30
    model = ApproachingModel(-0.01, -50, max_hor_acc * 0.5, 500, accuracy=100)
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        v_hor = v[1:3]
        err_hor = -r[1:3]
        dist = norm(err_hor)
        min_dist = model.find_dist_by_spd(norm(v_hor)) + 6. * norm(v_hor)
        if dist <= min_dist: break
        dir_hor = normalize(err_hor)
        v_hor_err = v[1:3] - dot(v[1:3], dir_hor) * dir_hor
        dir = -v + array((0., *v_hor_err)) * 5
        # print('\n', -v, dir, array((0., *v_hor_err)))
        rocket.update_ap(dir, roll)
        err_ang = rad2deg(vec_ang(v[1:3], dir_hor))
        debug('方向误差：%g度，水平速度误差：%s，%g %g'%(err_ang, str(v_hor_err), dist, min_dist))
    start = False
    dir = -rocket.velocity()
    dir2 = dir.copy()
    dir2[0] = 0.
    dir = vec_around(dir, dir2, radians(10))
    dir = vec_around(array([1, 0, 0]), dir, radians(30))
    min_spd = 1e9
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        err_hor = -r[1:3]
        dist = norm(err_hor)
        spd = norm(v[1:3])
        min_spd = min(min_spd - 0.05, spd)
        if spd < 5 or (spd < 20 and spd > min_spd):
            break
        dir2 = dir
        acc_hor2 = 0.
        if not start:
            min_dist = model.find_dist_by_spd(norm(v[1:3])) + 1. * norm(v_hor)
            start = dist <= min_dist
            acc = 0.
            if start:
                print('\nlanding burn1开始于高度：', r[0], '速度：', v)
        else:
            acc_hor = model.next_acc(err_hor, v[1:3], 0.5)
            acc_hor = vec_clamp(acc_hor, max_hor_acc)
            dir_hor = normalize(dir[1:3])
            acc_hor_proj = dot(acc_hor, dir_hor)
            acc1 = max(0., acc_hor_proj) * norm(dir) / norm(dir[1:3])
            dir2 = normalize(dir) * acc1
            if acc1 > 0.:
                acc_hor2 = (acc_hor - acc_hor_proj * dir_hor) * 0.3 # 水平分量垂直于dir的分量
                acc_hor2 = vec_clamp(acc_hor2, acc1 * 0.1)
                dir2 += array((0., *acc_hor2))
            acc = norm(dir2)
        debug('水平误差：%s，速度大小：%g，加速度： %s %s'%(str(err_hor), norm(v[1:3]), str(acc), str(acc_hor2)))
        rocket.update_ap(dir2)
        rocket.thrust(acc)
    rocket.thrust(0.)
    rocket.release_ap_control()

def landing_burn2(rocket: Rocket, rocket_height):
    rocket.ap.dir_model.lock_accuracy_ratio = 0.75
    g = rocket.orbit.body.surface_gravity
    max_ver_acc = min(30, rocket.available_thrust / rocket.mass - g)
    model = ApproachingModel(-0.01, -50, max_ver_acc * 0.8, 500, accuracy=3, lock_accuracy_ratio=0.1)
    final_landing_started = False
    rocket.ap.update_config(settling_time=0.5)
    while is_alive():
        v = rocket.velocity()
        r = rocket.position()
        err_ver = rocket_height - r[0]
        if abs(err_ver) < 10 and norm(v[0]) < 1.5: break
        if abs(err_ver) < 100:
            rocket.control.gear = True
        acc_ver = model.next_acc(err_ver, v[0], 0.3)
        acc_ver = clamp(acc_ver, -g, max_ver_acc) + g
        acc_hor = -r[1:3] * 0.1 - v[1:3] * 0.5
        if r[0] < 3000:
            if norm(acc_hor) < 3 and acc_ver == 0.:
                acc_hor *= 0.
            acc_ver = max(acc_ver, norm(acc_hor) * 1.5)
            acc = array(((acc_ver, *acc_hor)))
            acc = vec_clamp_yz(acc, radians(80))
            dir = acc
            if norm(acc) == 0.:
                dir = (1., 0, 0)
            debug(acc, r, v)
            rocket.update_ap(dir)
            acc = norm(acc)
            rocket.thrust(acc)
            if not final_landing_started:
                final_landing_started = acc > 0.
        if not final_landing_started:
            max_ang = 15
            min_ang = 3
            max_ang_height = 10000
            min_ang_height = 3000
            ang = (r[0] - min_ang_height) / (max_ang_height - min_ang_height) * (max_ang - min_ang) + min_ang
            ang = clamp(ang, min_ang, max_ang)
            dir_hor = (r[1:3] * 0.08 + v[1:3] * 0.5) * 0.5
            dir = array([max(norm(dir_hor) * 3, 10), *dir_hor])
            dir = vec_around(-v, dir, radians(ang))
            rocket.update_ap(dir)
            rocket.thrust(0.)
            debug(r[1:3], v[1:3])
    rocket.release_ap_control()
    rocket.thrust(0.)

def launch_and_booster_back_mission():
    def mission():
        vessel = space_center.active_vessel
        launch(Rocket(space_center, vessel), after_booster_seperation=booster_landing)
    
    task = Thread(target=mission)
    thread_queue.put(task)
    task.start()

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
        booster_back(rocket, myself_ready, another_ready, roll)
        landing_burn1(rocket, roll)
        landing_burn2(rocket, 16)
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
        entry_burn(rocket)
        landing_burn1(rocket)
        landing_burn2(rocket, 18.85)
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
    # first_stage_landing()
    # vessel = space_center.active_vessel
    # ref = surface_ref(vessel.orbit.body, *(-0.205703802260084, -74.4730931260344))
    # rocket = Rocket(space_center, vessel)
    # rocket.reset_ref(ref)
    # # landing_burn1(rocket)
    # landing_burn2(rocket)
    # launch_and_booster_back_mission()

    launch_and_first_stage_landing_mission()

    while True:
        try:
            if thread_queue.empty(): break
            task = thread_queue.get_nowait()
            task.join(0.5)
            if task.is_alive():
                thread_queue.put(task)
        except KeyboardInterrupt as e:
            mission_abort.set()