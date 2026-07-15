"""Regression tests for capsule collision (the disk-capsule pair shipped broken
once because nothing exercised it -- this makes sure that can't happen again)."""
import math
import pytest
from worlds import World, Body, Disk, Capsule, contact

def test_capsule_disk_reports_overlap():
    # a disk overlapping a horizontal capsule must produce exactly one contact
    # with the right penetration (this is the pair that had the cap. prefix bug)
    W = World(360, 40, 40)
    cap = Body(10, 10, shape=Capsule(2.0, 0.5))          # horizontal spine
    ball = Body(10, 9.2, shape=Disk(0.5))                # sitting 0.8 above -> overlap 0.2
    cs = contact(cap, ball)
    assert len(cs) == 1
    a, b, nx, ny, p, px, py = cs[0]
    assert p == pytest.approx(0.2, abs=1e-6)
    assert ny == pytest.approx(-1.0, abs=1e-6)           # normal points up toward the ball

def test_ball_rests_on_a_capsule_shelf():
    # drop a ball onto a static horizontal capsule -> it must land ON it,
    # not fall through to the floor
    W = World(360, 12, 20)                               # floor at y=10
    shelf = Body(10, 8.0, static=True, shape=Capsule(3.0, 0.4))
    W.add_body(shelf, "shelf")
    ball = Body(10, 5.0, mass=1, shape=Disk(0.5))
    W.add_body(ball, "ball")
    lowest = ball.y
    for _ in range(720):
        W.step()
        lowest = max(lowest, ball.y)                     # +y is down: track the deepest it gets
    # the ball may still be bouncing, but it must never sink past the shelf center (8.0);
    # if capsule-disk contact were broken it would tunnel through toward the floor-rest (9.5)
    assert lowest < 8.0

def test_flat_capsule_gives_two_contacts():
    # the manifold: a flat capsule pressed into the plane must report BOTH end-caps
    W = World(360, 20, 20)                               # floor at y=18
    cap = Body(10, 18 - 0.3, theta=0, shape=Capsule(2.0, 0.5))   # embedded 0.2
    W.add_body(cap, "cap")
    cs = contact(W.bodies["floor"], cap)
    assert len(cs) == 2
    assert all(c[4] == pytest.approx(0.2, abs=1e-9) for c in cs)  # equal penetration
    xs = sorted(c[5] for c in cs)
    assert xs[0] == pytest.approx(8.0) and xs[1] == pytest.approx(12.0)  # both ends

def test_tilted_capsule_self_levels():
    # dropped with a tilt, the two-contact base should right it (theta -> ~0)
    W = World(360, 20, 20)
    cap = Body(10, 14, theta=0.5, mass=1, shape=Capsule(2.0, 0.5))
    W.add_body(cap, "cap")
    for _ in range(360 * 4):
        W.step()
    assert abs(cap.theta) < 0.08
    assert abs(cap.omega) < 0.5

def test_capsule_capsule_reports_overlap():
    W = World(360, 40, 40)
    a = Body(10, 10, shape=Capsule(2.0, 0.5))
    b = Body(10, 9.2, shape=Capsule(2.0, 0.5))           # parallel, overlapping by 0.2
    cs = contact(a, b)
    assert len(cs) == 1
    assert cs[0][4] > 0                                  # positive penetration
