"""Watch any World play out.

Usage:
    from viewer import show
    from test_spin import make_ramp_world

    show(lambda: make_ramp_world(30, 1.0))   # callable -> R restarts the scenario
    show(my_world)                            # or pass a World directly

Accepts a World, a tuple containing a World (test helpers often return
(world, ball, ...)), or a zero-arg function returning either.

Controls:  SPACE pause   R restart (if given a callable)   ESC quit
"""
import math
import pygame
from worlds import World, Body, Disk, Plane

W_PX, H_PX = 900, 900
FPS = 60
BG = (255, 255, 255)
STATIC_COL = (30, 30, 30)
DISK_COL = (40, 90, 220)
HAND_COL = (220, 60, 60)
SPRING_COL = (200, 40, 40)
HUD_COL = (110, 110, 110)


def _extract_world(thing):
    if callable(thing):
        thing = thing()
    if isinstance(thing, World):
        return thing
    if isinstance(thing, (tuple, list)):
        for item in thing:
            if isinstance(item, World):
                return item
    raise TypeError("show() needs a World, a tuple containing one, or a callable returning one")


def _dynamic_bodies(world):
    return [b for b in world.bodies.values()
            if b.inv_mass > 0 and isinstance(b.shape, Disk)]


def _fit_camera(world):
    """Target camera (center, pixels-per-meter) framing the dynamic bodies."""
    dyn = _dynamic_bodies(world)
    if not dyn:
        return world.width / 2, world.height / 2, 50.0
    xs = [b.x for b in dyn]
    ys = [b.y for b in dyn]
    rmax = max(b.radius for b in dyn)
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    span_x = (max(xs) - min(xs)) + 4 * rmax + 2.0
    span_y = (max(ys) - min(ys)) + 4 * rmax + 2.0
    ppm = min(W_PX / span_x, H_PX / span_y)
    return cx, cy, min(max(ppm, 2.0), 200.0)


def show(scenario, sim_seconds=None):
    world = _extract_world(scenario)
    steps_per_frame = max(1, round((1 / world.dt) / FPS))

    pygame.init()
    screen = pygame.display.set_mode((W_PX, H_PX))
    pygame.display.set_caption("world viewer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    cam_x, cam_y, ppm = _fit_camera(world)
    t = 0.0
    paused = False
    running = True

    def to_screen(x, y):
        return (int((x - cam_x) * ppm + W_PX / 2),
                int((y - cam_y) * ppm + H_PX / 2))

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    paused = not paused
                if event.key == pygame.K_r and callable(scenario):
                    world = _extract_world(scenario)
                    cam_x, cam_y, ppm = _fit_camera(world)
                    t = 0.0

        if not paused:
            for _ in range(steps_per_frame):
                world.step()
            t += steps_per_frame * world.dt

        # ease the camera toward whatever the bodies are doing
        tx_, ty_, tppm = _fit_camera(world)
        cam_x += (tx_ - cam_x) * 0.08
        cam_y += (ty_ - cam_y) * 0.08
        ppm += (tppm - ppm) * 0.08

        screen.fill(BG)

        # planes: a long line through the anchor, along the surface tangent
        span_m = (W_PX + H_PX) / ppm
        for body in world.bodies.values():
            if isinstance(body.shape, Plane):
                sx, sy = body.shape.nx, body.shape.ny
                tanx, tany = -sy, sx
                p1 = to_screen(body.x - tanx * span_m, body.y - tany * span_m)
                p2 = to_screen(body.x + tanx * span_m, body.y + tany * span_m)
                pygame.draw.line(screen, STATIC_COL, p1, p2, 3)

        # springs
        for spring in world.springs.values():
            pygame.draw.line(screen, SPRING_COL,
                             to_screen(spring.body1.x, spring.body1.y),
                             to_screen(spring.body2.x, spring.body2.y), 2)

        # disks, with a clock hand showing theta so spin is visible
        for body in world.bodies.values():
            if isinstance(body.shape, Disk):
                centre = to_screen(body.x, body.y)
                r_px = max(2, int(body.radius * ppm))
                col = STATIC_COL if body.inv_mass == 0 else DISK_COL
                pygame.draw.circle(screen, col, centre, r_px, 0 if r_px < 4 else 2)
                hand = to_screen(body.x + body.radius * math.cos(body.theta),
                                 body.y + body.radius * math.sin(body.theta))
                pygame.draw.line(screen, HAND_COL, centre, hand, 2)

        hud = f"t = {t:6.2f} s    scale = {ppm:5.1f} px/m"
        if paused:
            hud += "    [PAUSED - space]"
        screen.blit(font.render(hud, True, HUD_COL), (12, 10))

        pygame.display.flip()
        clock.tick(FPS)

        if sim_seconds is not None and t >= sim_seconds:
            running = False

    pygame.quit()


if __name__ == "__main__":
    # default demo: two balls and a chain on the floor
    def demo():
        w = World(360, 15, 15)
        w.add_body(Body(4, 6, 2, 0, 3, mu=1.0, shape=Disk(0.5)), "ball")
        w.add_body(Body(10, 4, -1, 0, 3, mu=0.05, shape=Disk(0.5)), "ball2")
        return w
    show(demo)
