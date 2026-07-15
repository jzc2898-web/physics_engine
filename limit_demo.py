"""Watch the joint Limit at work: a capsule pendulum with hard angle stops.

Run:  python limit_demo.py

  SPACE ... kick the rod (alternates direction)
  L ....... toggle the limit on/off  (kick with it OFF to loop over the top)
  R ....... reset
  ESC ..... quit

The red rays show the allowed wedge. With the limit ON the rod slams into
an invisible stop at each ray; OFF, the same kick loops it over the top.
"""
import math
import pygame
import draw
from worlds import World, Body, Disk, Capsule, Joint, Limit

HANG = math.pi / 2                 # +y is down: hanging straight down = theta 90 deg
SWING = 0.4                        # allowed excursion each way (rad)
KICK = 8.0                         # omega that would loop over the top


def build():
    w = World(360, 20, 20, solver="impulse", joint_beta=0.5)
    anchor = Body(10, 5, static=True, shape=Disk(0.05))
    rod = Body(10, 6, mass=2, theta=HANG, shape=Capsule(1.0, 0.15))
    w.add_body(anchor, "anchor")
    w.add_body(rod, "rod")
    w.add_joint(Joint(anchor, rod, (0, 0), (-1.0, 0)), "pin")
    w.add_joint(Limit(anchor, rod, HANG - SWING, HANG + SWING), "stop")
    return w, anchor, rod


def main():
    pygame.init()
    screen = pygame.display.set_mode((900, 700))
    pygame.display.set_caption("joint limit -- SPACE kick, L toggle limit, R reset")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo", 18)

    world, anchor, rod = build()
    limit_on = True
    kick_dir = 1
    PPM, CAM = 60, (10, 8)

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_SPACE:
                    rod.omega += KICK * kick_dir
                    kick_dir = -kick_dir
                elif e.key == pygame.K_r:
                    world, anchor, rod = build()
                    limit_on = True
                elif e.key == pygame.K_l:
                    limit_on = not limit_on
                    if limit_on:
                        world.add_joint(Limit(anchor, rod, HANG - SWING, HANG + SWING), "stop")
                    else:
                        del world.joints["stop"]

        for _ in range(6):                     # 360 Hz sim at 60 fps
            world.step()

        screen.fill((255, 255, 255))
        # the allowed wedge: a red ray at each stop angle, drawn from the pin
        ax, ay = anchor.x, anchor.y
        for ang in (HANG - SWING, HANG + SWING):
            ex = ax + 2.4 * math.cos(ang)
            ey = ay + 2.4 * math.sin(ang)
            x1 = int((ax - CAM[0]) * PPM + 450); y1 = int((ay - CAM[1]) * PPM + 350)
            x2 = int((ex - CAM[0]) * PPM + 450); y2 = int((ey - CAM[1]) * PPM + 350)
            pygame.draw.line(screen, (220, 60, 60), (x1, y1), (x2, y2), 2)

        draw.draw_world(screen, world, ppm=PPM, camera=CAM)

        hud = (f"theta = {rod.theta:6.3f} rad   allowed {HANG-SWING:.3f} .. {HANG+SWING:.3f}   "
               f"limit: {'ON' if limit_on else 'OFF -- kick it over the top!'}")
        screen.blit(font.render(hud, True, (30, 30, 34)), (16, 12))
        screen.blit(font.render("SPACE kick   L toggle limit   R reset   ESC quit",
                                True, (120, 120, 126)), (16, 668))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
