from time import time
import krpc
import numpy as np
from time import sleep
from control.vec import *
from control.auto_pilot import AutoPilot

conn = krpc.connect('AP TEST2')
vessel = conn.space_center.active_vessel
ap = AutoPilot(lock_accuracy_ratio=0.9, settling_time=0.1)
ref = vessel.orbital_reference_frame
# ref = vessel.surface_reference_frame
# ref = ref.create_hybrid(vessel.orbit.body.reference_frame, rotation=ref)

def get_torque(vessel):
    tor1 = np.abs(vessel.available_reaction_wheel_torque[0])
    tor2 = np.abs(vessel.available_rcs_torque[0])
    tor3 = np.abs(vessel.available_engine_torque[0])
    tor4 = np.abs(vessel.available_control_surface_torque[0])
    return tor1 + tor2 + tor3 + tor4

while True:
    ang_v = vessel.angular_velocity(ref)
    ang_v = -array(ang_v)
    dir = vessel.direction(ref)
    target = (None, 0, 1, 0)
    
    x = array((1., 0, 0))
    x_ = normalize(dir)
    ang = vec_ang(x, x_)
    x_rot_axis = normalize(cross(x_, x))
    y_ = normalize(array(conn.space_center.transform_direction((0, 0, 1), vessel.reference_frame, ref)))
    y = rotate(x_rot_axis, y_, ang)
    ang1 = vec_ang(y, array((0., 1, 0)))
    ang2 = vec_ang(y, array((0., 0, 1)))
    roll = ang1
    if ang2 > pi / 2:
        roll = -roll
    cur = (roll, *dir)

    tor = get_torque(vessel)
    moi = array(vessel.moment_of_inertia)
    acc = tor / moi
    acc2 = (acc[1], acc[2], acc[0])
    ap.update_max_acc(acc2)
    x, y, z = ap.update(cur, target, ang_v, rot_flag=-1.)
    vessel.control.roll = x
    vessel.control.yaw = y
    vessel.control.pitch = z

    # conn.drawing.clear()
    # ang_v_dir = normalize(ang_v)
    # line = conn.drawing.add_line((0, 0, 0), tuple(ang_v_dir * (norm(ang_v) * 20)), ref)
    # line.thickness = 0.3
    # line.color = (1, 1, 0)
        
    # lines = [
    #     tuple(x_ * 20),
    #     tuple(y_ * 20),
    #     tuple(z_ * 20),
    #     tuple(target_dir * 30)
    # ]
    # for l, c in zip(lines, [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1)]):
    #     line = conn.drawing.add_line((0, 0, 0), l, ref)
    #     line.thickness = 0.1
    #     line.color = c
    sleep(0.05)

# ref = vessel.reference_frame

# vessel.control.yaw = 1


# while True:
#     conn.drawing.clear()
#     lines = [
#         (10, 0, 0),
#         (0, 10, 0),
#         (0, 0, 10)
#     ]
#     for l, c in zip(lines, [(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
#         line = conn.drawing.add_line((0, 0, 0), l, ref)
#         line.thickness = 0.3
#         line.color = c
#     ang_v = vessel.angular_velocity(vessel.surface_reference_frame)
#     ang_v = array(ang_v) * 10
#     print(ang_v)
#     line = conn.drawing.add_line((0, 0, 0), tuple(ang_v), vessel.surface_reference_frame)
#     line.thickness = 0.1
#     line.color = (1, 1, 0)
#     sleep(0.05)