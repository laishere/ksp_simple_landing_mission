from numpy import ndarray
from control.vec import *
from control.auto_pilot import AutoPilot
import krpc
from time import time, sleep

def surface_position(body, latitude, longitude, ref=None):
    if ref is None:
        ref = body.reference_frame
    biome = body.biome_at(latitude, longitude)
    if biome == 'Water':
        return body.msl_position(latitude, longitude, ref)
    return body.surface_position(latitude, longitude, ref)

def surface_ref(body, latitude, longitude):
    '''
    根据给定维度和经度构造地面参考系。
    仅确保x轴竖直向上，yz位于水平方向，不论角度
    '''
    ref = body.reference_frame
    pos = surface_position(body, latitude, longitude)
    x = array((1., 0, 0))
    target_dir = array(pos)
    ang = vec_ang(target_dir, x) / 2 # 半角
    dir = cross(target_dir, x) # 旋转轴
    rot_x = (cos(ang), *(sin(ang) * dir / norm(dir))) # 四元数
    return ref.create_relative(ref, pos, rotation=rot_x)

def vessel_surface_ref(vessel):
    body = vessel.orbit.body
    ref = body.reference_frame
    return ref.create_hybrid(ref, rotation=vessel.surface_reference_frame)

def find_vessel_by_name(space_center, name):
    for v in space_center.vessels:
        if v.name == name:
            return v

class Rocket:

    def __init__(self, space_center, vessel, ref=None) -> None:
        self.space_center = space_center
        self.vessel = vessel
        if ref is None:
            ref = vessel_surface_ref(vessel)
        self.ref = ref
        self.ap = self.get_ap()
        self.ap_config_updated_at = 0

    def get_ap(self) -> AutoPilot:
        return AutoPilot()

    def reset_ref(self, ref):
        self.ref = ref

    def position(self) -> ndarray:
        return array(self.vessel.position(self.ref))

    def velocity(self) -> ndarray:
        return array(self.vessel.velocity(self.ref))

    def _ap_auto_config(self):
        dt = time() - self.ap_config_updated_at
        if dt < 0.5: return
        v = self.vessel
        tor1 = np.abs(v.available_reaction_wheel_torque[0])
        tor2 = np.abs(v.available_rcs_torque[0])
        tor3 = np.abs(v.available_engine_torque[0])
        tor4 = np.abs(v.available_control_surface_torque[0])
        tor = tor1 + tor2 + tor3 + tor4
        moi = v.moment_of_inertia
        acc = tor / moi
        acc2 = (acc[1], acc[2], acc[0])
        self.ap.update_max_acc(acc2)
        self.ap_config_updated_at = time()

    def update_ap(self, dir, roll: float=None):
        self.last_dir = dir
        self.last_roll = roll
        self._ap_auto_config()
        x = self.vessel.direction(self.ref)
        y = array(self.space_center.transform_direction((0, 0, 1.), self.vessel.reference_frame, self.ref))
        x0 = array((1., 0, 0))
        rot_x = normalize(cross(x, x0))
        rot_x_ang = vec_ang(x, x0)
        y0 = rotate(rot_x, y, rot_x_ang)
        ang1 = vec_ang(y0, (0, 1, 0))
        ang2 = vec_ang(y0, (0, 0, 1))
        cur_roll = ang1
        if ang2 > pi / 2: cur_roll = -cur_roll
        cur = (cur_roll, *x)
        if dir is None:
            dir = x # 如果没有设置目标方向则以当前方向为目标方向，即维持当前方向稳定
        target = (roll, *dir)
        ang_vel = -array(self.vessel.angular_velocity(self.ref))
        ctrl_x, ctrl_y, ctrl_z = self.ap.update(cur, target, ang_vel, rot_flag=-1)
        ctrl = self.vessel.control
        ctrl.roll = ctrl_x
        ctrl.yaw = ctrl_y
        ctrl.pitch = ctrl_z

    def keep_last_ap(self):
        self.update_ap(self.last_dir, self.last_roll)

    def update_ap_and_wait(self, duration, dir, roll=None, dt=0.1):
        t = time()
        while time() - t < duration:
            self.update_ap(dir, roll)
            sleep(dt)

    def release_ap_control(self):
        ctrl = self.vessel.control
        ctrl.roll = 0.
        ctrl.pitch = 0.
        ctrl.yaw = 0.

    def thrust(self, acc: float):
        self.vessel.control.throttle = acc * self.vessel.mass / self.vessel.available_thrust

    def __getattribute__(self, name: str):
        try: attr = super().__getattribute__(name)
        except: attr = getattr(self.vessel, name)
        return attr

def ref_test():
    '''
    运行游戏，在停机坪绘制x、y、z轴验证参考系计算是否正确
    '''
    conn = krpc.connect(name='KSP TEST')
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    ref = surface_ref(body, -0.09680444444444444, -74.61740444444444)
    lines = [
        (100, 0, 0),
        (0, 100, 0),
        (0, 0, 100)
    ]
    for l in lines:
        line = conn.drawing.add_line((0, 0, 0), l, ref)
        line.thickness = 1
        line.color = l
    sleep(20)

def rocket_ap_test():
    '''
    在轨道上测试姿态控制
    '''
    conn = krpc.connect(name='KSP TEST')
    vessel = conn.space_center.active_vessel
    ref = vessel.orbital_reference_frame
    rocket = Rocket(conn.space_center, vessel, ref)
    dir = [
        (0, 1, 0),
        (0, -1, 0),
        (1, 0, 0),
        (-1, 0, 0),
        (0, 0, 1),
        (0, 0, -1),
    ]
    for d in dir:
        t = time()
        while time() - t < 6:
            rocket.update_ap(d)
            sleep(0.05)

if __name__ == '__main__':
    ref_test()
    # rocket_ap_test()