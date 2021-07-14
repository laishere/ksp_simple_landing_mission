import re
from threading import Event
import krpc
from time import sleep
from control.dynamics import ApproachingModel
from ksp import surface_ref, Rocket
from control.vec import *
from mission import booster_back, entry_burn, landing_burn1, landing_burn2

conn = krpc.connect('Tutorials')
space_center = conn.space_center
vessel = space_center.active_vessel

def control():
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1.
    sleep(3.)
    ap = vessel.auto_pilot
    ap.engage()
    ap.target_pitch = 60
    vessel.control.rcs = True
    sleep(1.)
    vessel.control.brakes = True
    sleep(1.)
    vessel.control.throttle = 0.
    sleep(1.)
    vessel.control.gear = True

def info():
    ref = vessel.orbit.body.reference_frame
    print(vessel.position(ref))
    print(vessel.velocity(ref))
    orbit = vessel.orbit
    print(orbit.apoapsis_altitude)
    print(orbit.periapsis_altitude)

def ap_bug():
    ap = vessel.auto_pilot
    ap.engage()
    ap.target_direction = (0, 1, 0)
    sleep(100)

def simple_landing():
    body = vessel.orbit.body
    ref = surface_ref(body, -0.09680444444444444, -74.61740444444444)
    vessel.control.rcs = True
    ap = vessel.auto_pilot
    ap.engage()
    ap.reference_frame = ref
    max_up_acc = 20.
    model = ApproachingModel(-0.01, -50, max_up_acc * 0.8, 500., accuracy=3, lock_accuracy_ratio=0.2)
    rocket_height = 17
    g = 9.81
    while True:
        r = vessel.position(ref)
        r = array(r)
        v = vessel.velocity(ref)
        v = array(v)
        err_x = rocket_height - r[0]
        print(err_x)
        if abs(err_x) < 10 and abs(v[0]) < 1: break
    #    if abs(err_x) < 100:
    #        vessel.control.gear = True
        acc_x = model.next_acc(err_x, v[0], 0.3)
        acc_x = clamp(acc_x, -g, max_up_acc) + g
        acc_yz = -r[1:3] * 0.1 - v[1:3] * 0.5
        if norm(acc_yz) > 3:
            acc_x2 = min(norm(acc_yz) * 3, 2 * g)
            acc_x = max(acc_x, acc_x2)
        if acc_x == 0.:
            acc_yz *= 0.
        acc = array([acc_x, *acc_yz])
        acc = vec_clamp_yz(acc, radians(80))
        ap.target_direction = (1, 0, 0) if norm(acc) == 0. else acc
        vessel.control.throttle = norm(acc) * vessel.mass / vessel.available_thrust
        sleep(0.05)
    vessel.control.throttle = 0.
    ap.disengage()
    vessel.control.sas = True
    

# control()
# info()
# ap_bug()
# simple_landing()

# rocket = Rocket(space_center, vessel)
# body = rocket.orbit.body

# ref = surface_ref(body, -0.185401781224271, -74.4729352356523)
# rocket.reset_ref(ref)
# myself_ready = Event()
# another_ready = Event()
# another_ready.set()
# booster_back(rocket, myself_ready, another_ready, None)
# landing_burn1(rocket)
# landing_burn2(rocket, 18.85)

# ref = surface_ref(body, -0.0933756132563131, -56.8000001513979)
# rocket.reset_ref(ref)
# entry_burn(rocket)
# landing_burn1(rocket)
# landing_burn2(rocket, 18.85)