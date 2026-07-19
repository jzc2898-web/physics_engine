"""Watch the trained crawler act. Loads actor.pt and renders one continuous run.

  R ..... reset the crawler
  ESC ... quit

Uses the policy's MEAN action (no sampling) so you see its learned gait cleanly.
"""
import pygame, torch
import ppo, draw
from ppo import Env, Actor

PPM = 55

def main():
    actor = Actor(23, 64, 16)
    actor.load_state_dict(torch.load("actor.pt"))
    actor.eval()

    env = Env()
    obs = env.reset()

    pygame.init()
    screen = pygame.display.set_mode((900, 800))
    pygame.display.set_caption("crawler  --  R reset, ESC quit")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("menlo", 18)

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_r:
                    obs = env.reset()

        with torch.no_grad():
            mean = actor(obs)              # deterministic: the policy's mean action
        obs, reward, done = env.step(mean)
        if done:
            obs = env.reset()

        trunk = env.world.bodies["trunk"]
        screen.fill((250, 250, 252))
        draw.draw_world(screen, env.world, ppm=PPM, camera=(trunk.x, trunk.y))
        screen.blit(font.render(f"trunk x = {trunk.x:6.2f} m", True, (30, 30, 40)), (16, 16))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
