from world import World, Body, Disk
import pygame

PPM = 50                 # pixels per meter
W_PX = 750
FPS = 60
WORLD_FPS = 360
FLOOR = 13.0             # floor height in meters

WHITE, BLACK = (255,255,255), (20,20,20)
BLUE, RED, GREY = (40,90,220), (200,40,40), (90,90,90)

def make_world():
    world = World(WORLD_FPS, FLOOR, W_PX/PPM, k=300, c=30)
    # left: a single ball -> one contact point -> bounces
    ball = Body(4, 6, 0, 0, 3, k=300, c=5, shape=Disk(0.3))
    world.add_body(ball, "ball")
    # right: a 'bone' = two masses + stiff damped spring, dropped TILTED -> flops
    a = Body(9,  6.0, 0, 0, 1.5, k=150, c=30, shape=Disk(0.3))
    b = Body(11, 4.8, 0, 0, 1.5, k=150, c=30, shape=Disk(0.3))
    world.add_body(a, "a"); world.add_body(b, "b")
    length = ((11-9)**2 + (6.0-4.8)**2) ** 0.5
    world.create_spring(0, 300, length, "a", "b", "bone",)
    world.springs["bone"].c = 10
    return world, ball, a, b

pygame.init()
screen = pygame.display.set_mode((W_PX, W_PX))
pygame.display.set_caption("one contact point bounces, two don't")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 30)

world, ball, a, b = make_world()
running, frame = True, 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            world, ball, a, b = make_world(); frame = 0

    for _ in range(WORLD_FPS // FPS):
        world.step()

    screen.fill(WHITE)
    pygame.draw.line(screen, BLACK, (0, FLOOR*PPM), (W_PX, FLOOR*PPM), 3)
    # ball
    pygame.draw.circle(screen, BLUE, (int(ball.x*PPM), int(ball.y*PPM)), int(ball.radius*PPM))
    # bone: link line + two end masses
    pygame.draw.line(screen, RED, (a.x*PPM, a.y*PPM), (b.x*PPM, b.y*PPM), 6)
    pygame.draw.circle(screen, BLACK, (int(a.x*PPM), int(a.y*PPM)), int(a.radius*PPM))
    pygame.draw.circle(screen, BLACK, (int(b.x*PPM), int(b.y*PPM)), int(b.radius*PPM))

    screen.blit(font.render("BALL  (small hop)", True, BLUE), (60, 20))
    screen.blit(font.render("BONE  (drops dead flat)", True, RED), (420, 20))
    screen.blit(font.render("press R to drop again", True, GREY), (60, 55))

    frame += 1
    if frame > 480:                      # auto-reset every ~8 s so it loops
        world, ball, a, b = make_world(); frame = 0

    pygame.display.flip()
    clock.tick(FPS)
pygame.quit()
