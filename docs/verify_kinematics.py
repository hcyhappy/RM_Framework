"""Standard-library verification for the convention in chassis-kinematics.md.
Run: python verify_kinematics.py. Numerical checks, not hardware validation.
"""
import math
import random


def local_velocity(vx, vy, omega, a, b):
    positions = [(a, b), (a, -b), (-a, b), (-a, -b)]
    return [(vx - omega*y, vy + omega*x) for x, y in positions]


def mecanum(vx, vy, omega, a, b):
    return [U + q*V for (U, V), q in
            zip(local_velocity(vx, vy, omega, a, b), [-1, 1, 1, -1])]


def omni(vx, vy, omega, a, b):
    c = 1/math.sqrt(2)
    # Directions are 135,45,-135,-45 degrees. Do not replace with wheel axle angles.
    return [c*(sx*U + sy*V) for (U, V), (sx, sy) in
            zip(local_velocity(vx, vy, omega, a, b), [(-1, 1), (1, 1), (-1, -1), (1, -1)])]


def swerve(vx, vy, omega, a, b):
    return [(math.hypot(U, V), math.atan2(V, U)) for U, V in
            local_velocity(vx, vy, omega, a, b)]


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def optimize(speed, target, current, previous_target=None):
    if abs(speed) < 1e-9:
        return 0.0, current if previous_target is None else previous_target
    error = wrap(target - current)
    if abs(error) > math.pi/2:
        speed = -speed
        error = wrap(target + math.pi - current)
    # Continuous module angle, closest to current angle.
    return speed, current + error


def near(a, b):
    assert math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-10), (a, b)


def check(vx, vy, w, a, b):
    m = mecanum(vx, vy, w, a, b)
    near(m[0]+m[1], m[2]+m[3])
    recovered = (sum(m)/4, (-m[0]+m[1]+m[2]-m[3])/4,
                 (-m[0]+m[1]-m[2]+m[3])/(4*(a+b)))
    for actual, expected in zip(recovered, (vx, vy, w)):
        near(actual, expected)
    o = omni(vx, vy, w, a, b)
    near(o[0]+o[3], o[1]+o[2])
    recovered = ((-o[0]+o[1]-o[2]+o[3])/(2*math.sqrt(2)),
                 (o[0]+o[1]-o[2]-o[3])/(2*math.sqrt(2)),
                 sum(o)/(4*(a+b)/math.sqrt(2)))
    for actual, expected in zip(recovered, (vx, vy, w)):
        near(actual, expected)
    for (speed, theta), (U, V) in zip(swerve(vx, vy, w, a, b),
                                     local_velocity(vx, vy, w, a, b)):
        near(speed*math.cos(theta), U)
        near(speed*math.sin(theta), V)
        current = random.uniform(-10*math.pi, 10*math.pi)
        s, t = optimize(speed, theta, current)
        near(s*math.cos(t), U)
        near(s*math.sin(t), V)
        assert abs(t-current) <= math.pi/2 + 1e-10


def main():
    random.seed(20261011)
    for vx, vy, w in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0), (1, .5, 1)]:
        check(vx, vy, w, .2, .15)
    for _ in range(1000):
        check(*(random.uniform(-3, 3) for _ in range(3)),
              random.uniform(.1, .5), random.uniform(.1, .5))
    speed, target = optimize(1, math.radians(170), math.radians(10))
    near(speed, -1); near(math.degrees(target), -10)
    near(math.degrees(wrap(math.radians(-179)-math.radians(179))), 2)
    near(optimize(0, 0, .8, previous_target=1.2)[1], 1.2)
    print('PASS: basic motions, 1000 random inverse/forward checks, steering equivalence and zero-speed policy')
    print('Sample: a=.20, b=.15, vx=1, vy=.5, omega=1; wheel order FL,FR,RL,RR')
    m=mecanum(1,.5,1,.2,.15);o=omni(1,.5,1,.2,.15);s=swerve(1,.5,1,.2,.15)
    for label, mi, oi, (si, angle) in zip(['FL','FR','RL','RR'],m,o,s):
        print(f'{label}: mecanum={mi:.6f} m/s; omni={oi:.6f} m/s; swerve={si:.6f} m/s, {math.degrees(angle):.6f} deg')


if __name__ == '__main__':
    main()
