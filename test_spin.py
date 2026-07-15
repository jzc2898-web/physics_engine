import math
import pytest
from worlds import World, Body, Disk, Plane

G = 9.81

def slope_speed(body, tx, ty):
    return body.x_vel*tx + body.y_vel*ty

def make_ramp_world(theta_deg, ball_mu):
    """Huge world so the auto-floor (at y=height) never interferes."""
    th = math.radians(theta_deg)
    world = World(360, 10000, 10000)
    nx, ny = math.sin(th), -math.cos(th)
    world.bodies["ramp"] = Body(x=0, y=500, x_vel=0, y_vel=0, mass=1, static=True,
                                shape=Plane(nx, ny), k=1e12, c=1e12, mu=0.8)
    # place ball exactly touching the ramp at the anchor: center = anchor + n*radius
    # (walk one radius OUT of the surface, along the outward normal)
    r = 0.5
    ball = Body(0 + nx*r, 500 + ny*r, 0, 0, 3, mu=ball_mu, shape=Disk(r))
    world.add_body(ball, "ball")
    # downhill tangent (points the way the ball will move)
    tx, ty = -ny, nx
    return world, ball, tx, ty

def measure_slope_accel(world, ball, tx, ty):
    # let the contact settle, then measure dv/dt along the slope
    for _ in range(180):            # 0.5 s settle
        world.step()
    v1 = slope_speed(ball, tx, ty)
    for _ in range(360):            # 1.0 s measure
        world.step()
    v2 = slope_speed(ball, tx, ty)
    return (v2 - v1) / 1.0

def test_slick_ball_slides_at_g_sin_theta():
    world, ball, tx, ty = make_ramp_world(30, ball_mu=1e-6)
    a = measure_slope_accel(world, ball, tx, ty)
    assert a == pytest.approx(G*math.sin(math.radians(30)), rel=0.05)   # ~4.905

def test_grippy_ball_rolls_at_two_thirds_g_sin_theta():
    world, ball, tx, ty = make_ramp_world(30, ball_mu=1.0)
    a = measure_slope_accel(world, ball, tx, ty)
    assert a == pytest.approx((2/3)*G*math.sin(math.radians(30)), rel=0.05)  # ~3.27
    # and it must actually be spinning (rolling, not sliding)
    assert abs(ball.omega) > 1.0

def test_opposite_spins_mesh_like_gears():
    # opposite spins meeting head-on: at the contact, both skins move the
    # same way at the same speed -> ZERO slip, like gear teeth meshing.
    # Friction has nothing to grab, so the spins must survive the bounce.
    world = World(360, 10000, 10000)
    a = Body(0.0, 500, 5, 0, 1, mu=1.0, omega=50,  shape=Disk(1))
    b = Body(2.5, 500, -5, 0, 1, mu=1.0, omega=-50, shape=Disk(1))
    world.add_body(a, "a"); world.add_body(b, "b")
    for _ in range(360):      # 1 s: approach, collide, separate
        world.step()
    assert a.omega == pytest.approx(50, rel=0.02)
    assert b.omega == pytest.approx(-50, rel=0.02)
    assert a.x_vel == pytest.approx(-b.x_vel, rel=1e-6)   # momentum symmetry

def test_same_spins_grind_and_slow():
    # SAME-sign spins meeting head-on: the touching skins move opposite ways
    # -> real slip -> friction bleeds spin from both and kicks them sideways.
    world = World(360, 10000, 10000)
    a = Body(0.0, 500, 5, 0, 1, mu=1.0, omega=50, shape=Disk(1))
    b = Body(2.5, 500, -5, 0, 1, mu=1.0, omega=50, shape=Disk(1))
    world.add_body(a, "a"); world.add_body(b, "b")
    for _ in range(360):
        world.step()
    assert abs(a.omega) < 50 and abs(b.omega) < 50        # grinding costs spin
    assert a.x_vel == pytest.approx(-b.x_vel, rel=1e-6)   # x-momentum conserved
    # spin converted into sideways motion: the y-velocities split apart
    assert abs(a.y_vel - b.y_vel) > 0.01

def test_rolling_turns_once_per_circumference():
    # a rolling ball's angle must track its travel: theta = distance / r
    world, ball, tx, ty = make_ramp_world(30, ball_mu=1.0)
    x0, y0 = ball.x, ball.y
    for _ in range(720):            # 2 s of rolling
        world.step()
    dist = math.hypot(ball.x - x0, ball.y - y0)
    assert ball.theta == pytest.approx(dist / ball.radius, rel=0.05)

def test_spin_bomb_scatters_the_pair():
    world = World(360, 15, 15)
    bottom = Body(7.5, 12.0, 0, 0, 1, mu=1.0, shape=Disk(1))     # resting on floor (height-2 = 13, minus radius 1)
    world.add_body(bottom, "bottom")
    for _ in range(360):                                          # let it settle
        world.step()
    top = Body(7.5, 9.5, 0, 0, 1, mu=1.0, omega=200, shape=Disk(1))
    world.add_body(top, "top")
    for _ in range(720):                                          # drop + kick, 2 s
        world.step()
    # the huge spin must fling them horizontally in OPPOSITE directions
    assert top.x_vel * bottom.x_vel < 0
    assert abs(top.x_vel) > 0.5 and abs(bottom.x_vel) > 0.5
    # and the top ball paid for it with spin
    assert abs(top.omega) < 200
