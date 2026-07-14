from world import World, Body, Disk, Plane
import pygame

PIXELS_XN = 750
METERS_PER_PIXEL = 50
FPS = 60
WORLD_FPS = 360
BLACK, WHITE = (0,0,0), (255, 255, 255)
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((PIXELS_XN, PIXELS_XN))
running = True
ball = Body(3, 3, 0, 0, 3, shape=Disk(.1))
ball2 = Body(7, 10, 0, 0, 3, shape=Disk(.1))
world = World(WORLD_FPS, PIXELS_XN/METERS_PER_PIXEL, PIXELS_XN/METERS_PER_PIXEL)
world.add_body(ball, "ball")
ramp = Body(x=0,y=5, shape=Plane(0.5, -0.866), static=True)
world.add_body(ramp, "ramp")
while running:
    screen.fill(WHITE)
    for ball in world.bodies.values():
        if ball.group is None and isinstance(ball.shape, Disk):
            pygame.draw.circle(screen, BLACK, (ball.x*METERS_PER_PIXEL, METERS_PER_PIXEL*ball.y), ball.radius*METERS_PER_PIXEL)
    for spring in world.springs.values():
        pygame.draw.line(screen, (200,40,40),
            (spring.body1.x*METERS_PER_PIXEL, spring.body1.y*METERS_PER_PIXEL),
            (spring.body2.x*METERS_PER_PIXEL, spring.body2.y*METERS_PER_PIXEL))
    floor_y = world.bodies["floor"].y * METERS_PER_PIXEL
    ramp = world.bodies["ramp"]
    pygame.draw.line(screen, BLACK, (0,floor_y), (PIXELS_XN, floor_y), 3)
    tx, ty = -ramp.shape.ny, ramp.shape.nx
    L = 100    # meters — comfortably past both screen edges
    pygame.draw.line(screen, BLACK,
        ((ramp.x - tx*L)*METERS_PER_PIXEL, (ramp.y - ty*L)*METERS_PER_PIXEL),
        ((ramp.x + tx*L)*METERS_PER_PIXEL, (ramp.y + ty*L)*METERS_PER_PIXEL), 3)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    for _ in range(WORLD_FPS//FPS):
        world.step()
    clock.tick(FPS)
    pygame.display.flip()
pygame.quit()