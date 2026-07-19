"""Manually flex the torque-activator pairs (hips/shoulders) and watch for oscillation.

Torso is pinned STATIC so one limb's settling behaviour is isolated.

  TAB ........ cycle which muscle is selected
  UP / DOWN .. change the selected muscle's activation (persists)
  0 .......... relax the selected muscle
  R .......... reset
  ESC ........ quit

If a limb driven to an angle keeps swinging back and forth instead of
settling, that's the missing angular damping in the torque_activator branch.
"""
import sys
sys.path.append("/Users/jcole27/walker")
import pygame
from worlds import World
from arm import make_human
import draw

PPM, CAM = 55, (7.5, 10.0)
TORQUE_MUSCLES = ["Rhip_f", "Rhip_e", "Lhip_f", "Lhip_e",
                  "Rsh_f", "Rsh_e", "Lsh_f", "Lsh_e"]


def build():
    w = World(360, 15, 15, solver="impulse", iters=20)
    make_human(w, static=True)          # trunk pinned -> isolate the limb
    return w


def main():
    pygame.init()
    screen = pygame.display.set_mode((900, 800))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo", 18)

    world = build()
    effort = {m: 0.0 for m in TORQUE_MUSCLES}
    sel = 0

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_TAB:
                    sel = (sel + 1) % len(TORQUE_MUSCLES)
                elif e.key == pygame.K_UP:
                    m = TORQUE_MUSCLES[sel]
                    effort[m] = min(1.0, round(effort[m] + 0.1, 2))
                elif e.key == pygame.K_DOWN:
                    m = TORQUE_MUSCLES[sel]
                    effort[m] = max(0.0, round(effort[m] - 0.1, 2))
                elif e.key == pygame.K_0:
                    effort[TORQUE_MUSCLES[sel]] = 0.0
                elif e.key == pygame.K_r:
                    world = build()
                    effort = {m: 0.0 for m in TORQUE_MUSCLES}

        for m in TORQUE_MUSCLES:
            world.springs[m].set_activation(effort[m])

        for _ in range(6):
            world.step()

        screen.fill((250, 250, 252))
        draw.draw_world(screen, world, ppm=PPM, camera=CAM)

        for i, m in enumerate(TORQUE_MUSCLES):
            mark = ">" if i == sel else " "
            col = (200, 40, 40) if i == sel else (90, 90, 96)
            bar = "#" * int(effort[m] * 20)
            screen.blit(font.render(f"{mark} {m:7} {effort[m]:.1f} |{bar:<20}|", True, col),
                        (16, 16 + 22 * i))
        screen.blit(font.render("TAB select   UP/DOWN effort   0 relax   R reset   ESC quit",
                                True, (120, 120, 126)), (16, 770))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
