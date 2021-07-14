from .vec import *
from .dynamics import ApproachingModel

class AutoPilot:

    def __init__(self, max_acc: tuple=None, max_spd: float=radians(360), accuracy=radians(1), settling_time=0.5, lock_accuracy_ratio=0.85) -> None:
        self.settling_time = settling_time
        self.dir_model = ApproachingModel(-1e-3, -100, 0, max_spd, accuracy, lock_accuracy_ratio)
        self.roll_model = ApproachingModel(-1e-3, 0, 0, max_spd, accuracy, lock_accuracy_ratio)
        self.max_acc_ratio = 1.0
        self.max_acc = max_acc
        self.update_max_acc(max_acc)

    def update_max_acc(self, max_acc: tuple):
        if max_acc is None: return
        self.max_acc = max_acc
        self.dir_model.max_acc = norm(self.max_acc[1:3])
        self.roll_model.max_acc = self.max_acc[0]

    def update_config(self, max_acc: tuple=None, settling_time=None):
        if max_acc is not None:
            self.update_max_acc(max_acc)
        if settling_time is not None:
            self.settling_time = settling_time

    def update(self, cur, target, angular_velocity, rot_flag=1., debug=False):
        roll = cur[0]
        target_roll = target[0] or 0.
        dir = array(cur[1:4])
        target_dir = array(target[1:4])
        angular_velocity = array(angular_velocity)
        x = array((1., 0, 0))
        y = array((0., cos(roll), sin(roll)))
        x_ = normalize(dir)
        ang = vec_ang(x, x_)
        x_rot_axis = normalize(cross(x, x_))
        y_ = rotate(x_rot_axis, y, ang)
        z_ = cross(x_, y_)
        dir_ang = vec_ang(dir, target_dir)
        dir_rot_axis = rot_flag * cross(dir, target_dir)
        if norm(dir_rot_axis) == 0.: # 当前方向与目标方向共线
            dir_rot_axis = y_
        dir_rot_axis = normalize(dir_rot_axis)
        dir_err = dir_rot_axis * dir_ang # 当存在角度误差时，我们希望角速度也是沿着旋转轴正方向，所以误差无需负号
        v_x = dot(angular_velocity, x_) * x_
        v_yz = angular_velocity - v_x
        acc_yz = self.dir_model.next_acc(dir_err, v_yz, self.settling_time)
        # acc_yz = -v_yz / self.settling_time
        if target[0] is None:
            # 如果目标旋转不指定则消除旋转角速度
            acc_x = -v_x / self.settling_time
        else:
            roll_err = target_roll - roll
            v_x_proj = dot(v_x, x_)
            if roll_err > pi:
                roll_err -= pi * 2
            elif roll_err < -pi:
                roll_err += pi * 2
            roll_err *= rot_flag
            acc_x = self.roll_model.next_acc(roll_err, v_x_proj, self.settling_time) * x_
        # 确保计算的角加速度分量在各自旋转轴上并转化为与最大加速度的比例值
        acc_level_x = dot(acc_x, x_) / self.max_acc[0]
        acc_level_y = dot(acc_yz, y_) / self.max_acc[1]
        acc_level_z = dot(acc_yz, z_) / self.max_acc[2]
        if not debug:
            return acc_level_x, acc_level_y, acc_level_z
        return acc_level_x, acc_level_y, acc_level_z, x_, y_, z_, target_dir

