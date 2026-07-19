"""Draw any World into any pygame surface, one call per frame.

Usage:
    import pygame, draw
    from world import World, ...

    screen = pygame.display.set_mode((900, 700))
    ...
    while running:
        world.step()                      # (or several steps)
        screen.fill((255, 255, 255))
        draw.draw_world(screen, world, ppm=40)
        pygame.display.flip()

draw_world(screen, world, ppm, camera):
    ppm    = pixels per meter (zoom)
    camera = (cx, cy) world point to center the screen on; defaults to the
             middle of the world box.
Draws every body by its shape (Disk / Capsule / Plane), all springs and
muscles, and a green dot at every joint anchor so you can see the pins.
"""
import math
import pygame
from worlds import Disk, Capsule, Plane

STATIC_COL  = (40, 40, 46)
DISK_COL    = (40, 90, 220)
CAPSULE_COL = (60, 130, 90)
SPINE_COL   = (255, 255, 255)
SPRING_COL  = (200, 40, 40)
JOINT_COL   = (30, 180, 30)


def draw_world(screen, world, ppm=40, camera=None, show_joints=False):
    w_px, h_px = screen.get_size()
    if camera is None:
        camera = (world.width / 2, world.height / 2)
    cx, cy = camera

    def to_px(x, y):
        return (int((x - cx) * ppm + w_px / 2),
                int((y - cy) * ppm + h_px / 2))

    # planes first (background)
    for body in world.bodies.values():
        if isinstance(body.shape, Plane):
            nx, ny = body.shape.nx, body.shape.ny
            tx, ty = -ny, nx                       # tangent along the surface
            L = (w_px + h_px) / ppm                # long enough to cross the screen
            pygame.draw.line(screen, STATIC_COL,
                             to_px(body.x - tx * L, body.y - ty * L),
                             to_px(body.x + tx * L, body.y + ty * L), 3)

    # springs / muscles (under the bodies)
    for spring in world.springs.values():
        if hasattr(spring, "off1"):                # muscle: draw between its pins
            p1 = spring._world(spring.body1, spring.off1)
            p2 = spring._world(spring.body2, spring.off2)
        else:                                      # plain spring: center to center
            p1 = (spring.body1.x, spring.body1.y)
            p2 = (spring.body2.x, spring.body2.y)
        pygame.draw.line(screen, SPRING_COL, to_px(*p1), to_px(*p2), 2)

    # disks and capsules
    for body in world.bodies.values():
        shape = body.shape
        if isinstance(shape, Capsule):
            A, B = shape.endpoints(body)
            a_px, b_px = to_px(*A), to_px(*B)
            r_px = max(1, int(shape.radius * ppm))
            col = STATIC_COL if body.inv_mass == 0 else CAPSULE_COL
            pygame.draw.line(screen, col, a_px, b_px, 2 * r_px)  # body
            pygame.draw.circle(screen, col, a_px, r_px)          # round caps -> capsule
            pygame.draw.circle(screen, col, b_px, r_px)
            pygame.draw.line(screen, SPINE_COL, a_px, b_px, 1)   # spine shows theta
        elif isinstance(shape, Disk):
            c_px = to_px(body.x, body.y)
            r_px = max(2, int(shape.radius * ppm))
            col = STATIC_COL if body.inv_mass == 0 else DISK_COL
            pygame.draw.circle(screen, col, c_px, r_px)
            hand = to_px(body.x + shape.radius * math.cos(body.theta),
                         body.y + shape.radius * math.sin(body.theta))
            pygame.draw.line(screen, SPINE_COL, c_px, hand, 2)   # spin hand

    # joint pins on top (debug view): draw_world(..., show_joints=True)
    if show_joints:
        for jnt in world.joints.values():
            if hasattr(jnt, "off1"):
                p = jnt.body1._world_point(*jnt.off1)
                pygame.draw.circle(screen, JOINT_COL, to_px(*p), 4)
