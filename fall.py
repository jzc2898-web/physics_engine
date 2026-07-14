from world import World, Body, Disk
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
ball = Body(3, 10, 0, 0, 3, shape=Disk(.1))
ball2 = Body(7, 10, 0, 0, 3, shape=Disk(.1))
world = World(WORLD_FPS, PIXELS_XN/METERS_PER_PIXEL, PIXELS_XN/METERS_PER_PIXEL)
world.add_body(ball, "ball")
world.add_body(ball2, "ball2")
world.create_spring(10, 30, 3, "ball", "ball2", "spring", mass=1.0)
while running:
    screen.fill(WHITE)
    for ball in world.bodies.values():
        if ball.group is None:
            pygame.draw.circle(screen, BLACK, (ball.x*METERS_PER_PIXEL, METERS_PER_PIXEL*ball.y), ball.radius*METERS_PER_PIXEL)
    for spring in world.springs.values():
        pygame.draw.line(screen, (200,40,40),
            (spring.body1.x*METERS_PER_PIXEL, spring.body1.y*METERS_PER_PIXEL),
            (spring.body2.x*METERS_PER_PIXEL, spring.body2.y*METERS_PER_PIXEL))

           
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    for _ in range(WORLD_FPS//FPS):
        world.step()
    clock.tick(FPS)
    pygame.display.flip()
pygame.quit()