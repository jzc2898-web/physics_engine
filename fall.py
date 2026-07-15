from worlds import World, Body, Disk, Plane, Capsule, Joint
import pygame, draw
PIXELS_XN = 750
METERS_PER_PIXEL = 50
FPS = 60
WORLD_FPS = 360
BLACK, WHITE = (0,0,0), (255, 255, 255)
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((PIXELS_XN, PIXELS_XN))
running = True
ball = Body(9.5, 6.5, static=True, shape=Disk(0.05))
world = World(WORLD_FPS, PIXELS_XN/METERS_PER_PIXEL, PIXELS_XN/METERS_PER_PIXEL, solver="impulse")
world.add_body(ball, "ball")
world.add_rope(7, 5, 12, 5, 8, 0.06, 3, "rope")
ramp = Body(x=0,y=5, shape=Plane(0.5, -0.866), static=True)
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