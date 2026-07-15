"""Live pygame viewer for the capsule playground.

Watch each interaction happen in real time. Standalone -- imports the engine
and the scene builders; modifies nothing.

Run:  python capsule_pygame.py

Controls:
  SPACE ....... pause / resume
  R ........... reset current scene
  <- / -> ..... previous / next scene
  UP / DOWN ... faster / slower (sim speed)
  . (period) .. single-step one frame while paused
  ESC / Q ..... quit
"""
import math
import pygame

from worlds import Disk, Capsule, Plane
from capsule_playground import (FPS, scene_hit_ball, scene_floor, scene_ramp,
                                scene_cap_cap)

SCREEN_W, SCREEN_H = 1100, 640
BG = (245, 245, 247)
GRID = (232, 232, 236)
PLANE_COL = (90, 90, 96)
WHITE = (255, 255, 255)
TEXT = (30, 30, 34)

def hexrgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))

# each entry: (builder, needs no args) -> returns (world, actors, seconds, title)
SCENES = [
    lambda: scene_hit_ball(False),
    lambda: scene_hit_ball(True),
    scene_floor,
    scene_ramp,
    scene_cap_cap,
]

class Camera:
    """Maps world metres -> screen pixels. +y is DOWN in both, so no flip."""
    def __init__(self, world):
        margin = 0.92
        self.scale = min(SCREEN_W/world.width, SCREEN_H/world.height) * margin
        # centre the world box in the window
        self.ox = (SCREEN_W - world.width*self.scale)/2
        self.oy = (SCREEN_H - world.height*self.scale)/2
    def px(self, x, y):
        return int(self.ox + x*self.scale), int(self.oy + y*self.scale)
    def r(self, r):
        return max(1, int(r*self.scale))

def draw_body(surf, cam, body, color):
    shape = body.shape
    if isinstance(shape, Plane):
        nx, ny = shape.nx, shape.ny
        tx, ty = -ny, nx                              # tangent along the surface
        L = 400
        a = cam.px(body.x - tx*L, body.y - ty*L)
        b = cam.px(body.x + tx*L, body.y + ty*L)
        pygame.draw.line(surf, PLANE_COL, a, b, 4)
    elif isinstance(shape, Capsule):
        A, B = shape.endpoints(body)
        Ap, Bp = cam.px(*A), cam.px(*B)
        rp = cam.r(shape.radius)
        pygame.draw.line(surf, color, Ap, Bp, 2*rp)   # body (butt-capped rectangle)
        pygame.draw.circle(surf, color, Ap, rp)       # + round end-caps -> stadium
        pygame.draw.circle(surf, color, Bp, rp)
        pygame.draw.line(surf, WHITE, Ap, Bp, 1)      # spine = orientation
    else:  # Disk
        c = cam.px(body.x, body.y)
        rp = cam.r(shape.radius)
        pygame.draw.circle(surf, color, c, rp)
        hx = body.x + shape.radius*math.cos(body.theta)
        hy = body.y + shape.radius*math.sin(body.theta)
        pygame.draw.line(surf, WHITE, c, cam.px(hx, hy), 2)   # spin hand

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("capsule playground -- live")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo", 20)
    small = pygame.font.SysFont("menlo", 15)

    idx = 0
    def load(i):
        world, actors, seconds, title = SCENES[i]()
        cam = Camera(world)
        colors = {name: hexrgb(c) for name, c in actors.items()}
        return world, cam, colors, seconds, title, 0.0    # last = sim time

    world, cam, colors, seconds, title, t = load(idx)
    steps_per_frame = FPS // 60          # real-time at 60 fps
    paused = False

    running = True
    while running:
        step_once = False
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif e.key == pygame.K_SPACE:
                    paused = not paused
                elif e.key == pygame.K_r:
                    world, cam, colors, seconds, title, t = load(idx)
                elif e.key == pygame.K_RIGHT:
                    idx = (idx + 1) % len(SCENES)
                    world, cam, colors, seconds, title, t = load(idx)
                elif e.key == pygame.K_LEFT:
                    idx = (idx - 1) % len(SCENES)
                    world, cam, colors, seconds, title, t = load(idx)
                elif e.key == pygame.K_UP:
                    steps_per_frame = min(24, steps_per_frame + 1)
                elif e.key == pygame.K_DOWN:
                    steps_per_frame = max(1, steps_per_frame - 1)
                elif e.key == pygame.K_PERIOD:
                    step_once = True

        if not paused or step_once:
            for _ in range(steps_per_frame):
                world.step()
                t += world.dt
            if t > seconds + 2.0:                      # auto-loop so the action replays
                world, cam, colors, seconds, title, t = load(idx)

        # ---- draw ----
        screen.fill(BG)
        for gx in range(0, SCREEN_W, 40):
            pygame.draw.line(screen, GRID, (gx, 0), (gx, SCREEN_H))
        for gy in range(0, SCREEN_H, 40):
            pygame.draw.line(screen, GRID, (0, gy), (SCREEN_W, gy))
        for name, body in world.bodies.items():
            draw_body(screen, cam, body, colors.get(name, (120, 120, 128)))

        screen.blit(font.render(f"[{idx+1}/{len(SCENES)}]  {title}", True, TEXT), (16, 12))
        hud = f"t={t:4.1f}s   speed={steps_per_frame}x/frame   {'PAUSED' if paused else 'running'}"
        screen.blit(small.render(hud, True, TEXT), (16, 40))
        screen.blit(small.render("SPACE pause  R reset  <- -> scene  UP/DOWN speed  . step  ESC quit",
                                 True, (110, 110, 116)), (16, SCREEN_H - 24))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
