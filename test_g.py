import pytest
from world import World, Body

def test_gravity():
    world = World(20, 20, 20)
    body = Body(world.height//2, 0, 0, 0,1)
    for _ in range(20):
        world.app_grav(body)
    assert body.x == world.height//2
    assert body.x_vel == 0
    assert body.y_vel == pytest.approx(9.81)
    assert body.y == (pytest.approx(1/2*9.81*((1+world.dt))))
def test_galileo():
    world = World(20, 20, 20)
    body = Body(world.height//2, 0, 0, 0, 1)
    body2 = Body(5, 0, 0, 0, 105)
    for _ in range(20):
        world.app_grav(body)
        world.app_grav(body2)
    assert body.y == pytest.approx(body2.y)
    assert body.y_vel == pytest.approx(body2.y_vel)
def test_spring():
    world = World(20,20,20)
    body = Body(x=world.height//2, y=15, x_vel=0, y_vel=0, mass=12)
    for _ in range(3000):
        world.app_grav(body)
    assert body.y == pytest.approx(body.mass*9.81/world.k+world.height)
