import math
from viewer import show
from test_spin import make_ramp_world
from test_g import test_galileo
from world import World, Body, Disk, Plane


def make_impulse_ramp(theta_deg=25, e=0.6):
    """Ball DROPPED onto a ramp, impulse solver. Watch the bounces reflect
    off the slope and shrink by e each hit."""
    th = math.radians(theta_deg)
    world = World(360, 10000, 10000, solver="impulse")
    nx, ny = math.sin(th), -math.cos(th)          # outward normal of the ramp
    world.bodies["ramp"] = Body(x=0, y=500, mass=1, static=True,
                                shape=Plane(nx, ny), e=0.8)
    # start the ball 4 m above the ramp surface, straight out along the normal
    r = 0.5
    ball = Body(0 + nx*(r + 4.0), 500 + ny*(r + 4.0), 0, 0, 3,
                e=e, mu=1.0, shape=Disk(r))
    world.add_body(ball, "ball")
    return world, ball


# penalty ramp (the old rolling test): ball ROLLS, spin hand turns
# show(lambda: make_ramp_world(30, 1.0))

# impulse ramp: ball BOUNCES down the slope
show(lambda: make_impulse_ramp(25, 0.6))
