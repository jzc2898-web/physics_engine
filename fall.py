from worlds import World, Body, Disk, Plane, Capsule, Joint
import pygame, draw, sys
sys.path.append("/Users/jcole27/walker")
from arm import make_human
PIXELS_XN = 750
METERS_PER_PIXEL = 50
FPS = 60
WORLD_FPS = 360
BLACK, WHITE = (0,0,0), (255, 255, 255)
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((PIXELS_XN, PIXELS_XN))
running = True
world = World(WORLD_FPS, PIXELS_XN/METERS_PER_PIXEL, PIXELS_XN/METERS_PER_PIXEL, solver="impulse")
ramp = Body(x=0,y=5, shape=Plane(0.5, -0.866), static=True)
make_human(world)
while running:
    screen.fill(WHITE)
    draw.draw_world(screen, world, show_joints=False)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    for _ in range(WORLD_FPS//FPS):
        world.step()
    clock.tick(FPS)
    pygame.display.flip()
pygame.quit()