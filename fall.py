from world import World, Body
import pygame

PIXELS_XN = 1000
METERS_PER_PIXEL = 50
FPS = 1000
BLACK, WHITE = (0,0,0), (255, 255, 255)
pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((PIXELS_XN, PIXELS_XN))
running = True
ball = Body((PIXELS_XN/METERS_PER_PIXEL)//2, 3.0, 0, 0, 1)
world = World(FPS, PIXELS_XN/METERS_PER_PIXEL, PIXELS_XN/METERS_PER_PIXEL)
while running:
    screen.fill(WHITE)
    pygame.draw.circle(screen, BLACK, (ball.x*METERS_PER_PIXEL, METERS_PER_PIXEL*ball.y), 5)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    world.app_grav(ball)
    clock.tick(FPS)
    pygame.display.flip()
pygame.quit()