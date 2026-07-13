import pytest
from world import World, Body
import math
def test_gravity():
    world = World(20, 20, 20)
    body = Body(world.height//2, 0, 0, 0,1)
    world.add_body(body, "ball")
    for _ in range(20):
        world.step()
    assert body.x == world.height//2
    assert body.x_vel == 0
    assert body.y_vel == pytest.approx(9.81)
    assert body.y == (pytest.approx(1/2*9.81*((1+world.dt))))
def test_galileo():
    world = World(20, 20, 20)
    body = Body(world.height//2, 0, 0, 0, 1)
    body2 = Body(5, 0, 0, 0, 105)
    world.add_body(body, "body")
    world.add_body(body2, "body2")
    for _ in range(20):
        world.step()
    assert body.y == pytest.approx(body2.y)
    assert body.y_vel == pytest.approx(body2.y_vel)
def test_spring():
    world = World(20,20,20)
    body = Body(x=world.height//2, y=15, x_vel=0, y_vel=0, mass=12)
    world.add_body(body, "body")
    for _ in range(3000):
        world.step()
    assert body.y == pytest.approx(body.mass*9.81/world.k + world.height - body.radius)
def test_ball_collision():
    world = World(20, 20, 20)
    ball1 = Body(x=0, y=0, x_vel=0, y_vel=0, mass=1)
    ball2 = Body(x=1, y=1, x_vel=0, y_vel=0, mass=1)
    world.ball_interact(ball1, ball2)
    assert ball2.fx == pytest.approx(ball2.fy)
    assert ball1.fx == pytest.approx(-ball2.fx)
    assert ball1.fy == pytest.approx(-ball2.fy)
def test_projectile_follows_discrete_law():
    G = 9.81
    world = World(FPS=1000, height=100000, width=100000, k=500, c=0)
    a = Body(500.0, 500.0, 0, 0, 1)   # gets ejected up-left at a weird angle
    b = Body(500.6, 500.8, 0, 0, 1)
    world.add_body(a, "a")
    world.add_body(b, "b")

    # run until the collision has fully ejected them; capture the launch state
    for _ in range(20000):
        world.step()
        if math.dist((a.x, a.y), (b.x, b.y)) > a.radius + b.radius:
            y0, v0 = a.y, a.y_vel
            break
    assert v0 < 0   # sanity: it really was launched upward

    # closed form of OUR discrete law: v(m) = v0 + m*G*dt,
    # y(m) = y0 + v0*dt*m + 1/2*G*dt^2*m*(m+1)  ->  peak at the last rising step
    m_star = math.floor(-v0 / (G * world.dt))
    y_pred = y0 + v0*world.dt*m_star + 0.5*G*world.dt**2*m_star*(m_star + 1)

    ys = []
    for _ in range(m_star + 50):          # run just past the peak
        world.step()
        ys.append(a.y)

    assert min(ys) == pytest.approx(y_pred, abs=1e-9)   # right height...
    assert ys.index(min(ys)) + 1 == m_star              # ...at the right time