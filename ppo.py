from worlds import Capsule, Plane, Disk, World, Body, Spring
import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "walker"))
from muscle import Muscle
from arm import make_human
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy
import copy

class Actor(nn.Module):
    def __init__(self, inpt, n_embd, oupt):
        super().__init__()
        self.block = nn.Sequential(
            nn.Linear(inpt, n_embd),
            nn.Tanh(),
            nn.Linear(n_embd, n_embd),
            nn.Tanh(),
            nn.Linear(n_embd, oupt)
        )
        self.lv = nn.Parameter(torch.zeros(oupt), requires_grad = True)
    def forward(self, x):
        x = self.block(x)
        return x
    def dist_builder(self, obs):
        mu = self.block(obs)
        sigma = torch.exp(self.lv)
        normal = torch.distributions.Normal(mu, sigma)
        return normal
class Critic(nn.Module):
    def __init__(self, inpt, n_embd):
        super().__init__()
        self.block = nn.Sequential(
            nn.Linear(inpt, n_embd),
            nn.Tanh(),
            nn.Linear(n_embd, n_embd),
            nn.Tanh(),
            nn.Linear(n_embd, 1)
        )
    def forward(self, x):
        x = self.block(x)
        return x
class Env():
    def __init__(self, fps=60, max_steps=1000):
        self.reset()
        self.max_steps = max_steps
        self.contact_penalty = 0.03      # per forbidden body touching the ground, per step
        # only feet may touch (hands/ulnas are free: no penalty, no reward)
        self.forbidden = ["trunk", "Rfemur", "Lfemur", "Rtibia", "Ltibia", "Rhumerus", "Lhumerus"]
        self.clean_bonus = 0.02          # bonus when ONLY feet (or nothing) touch the floor
    def step(self, action):
        acts = torch.clip(action, 0, 1).tolist()   # floats, not tensors -> physics stays in float math
        prev = self.world.bodies["trunk"].x
        for ac, mu in zip(acts, self.world.muscle_names):
            self.world.springs[mu].set_activation(ac)
        for _ in range(6):
            self.world.step()
        self.steps += 1
        obs = self.get_obs()
        rewards = self.get_reward(prev)
        if self.steps >= self.max_steps:
            return obs, rewards, True
        return obs, rewards, False
    def get_reward(self, prev):
        n_trunk = self.world.bodies["trunk"]
        rewards = 0
        rewards += n_trunk.x - prev
        for name in self.forbidden:                   # only feet may bear on the floor
            if self.world.bodies[name] in self.world.contact_bodies:
                rewards -= self.contact_penalty
        touching = self.world.contact_bodies
        clean = not any(self.world.bodies[n] in touching
                        for n in self.forbidden + ["Rulna", "Lulna"])
        if clean:                                     # feet-or-nothing stance: extra pay
            rewards += self.clean_bonus
        return rewards

    def reset(self):
        self.world = World(360, 15, 15, solver = "impulse", iters = 10)   # 360Hz: small dt so limbs can't tunnel through the floor
        human = make_human(self.world)
        obs = self.get_obs(True)
        self.steps = 0
        return obs
    def get_obs(self, first=False):
        obs = []
        pairs = [("Rfemur","trunk"), ("Lfemur","trunk"),
                ("Rhumerus","trunk"), ("Lhumerus","trunk"),
                ("Rtibia","Rfemur"), ("Ltibia","Lfemur"),
                ("Rulna","Rhumerus"), ("Lulna","Lhumerus"),
                ("Rfoot","Rtibia"), ("Lfoot","Ltibia")]
        for child, parent in pairs:
            c, p = self.world.bodies[child], self.world.bodies[parent]
            obs.append(c.theta - p.theta)
            obs.append(c.omega - p.omega)
        trunk = self.world.bodies["trunk"]
        obs.extend([trunk.theta, trunk.x_vel, trunk.y_vel])
        if first:
            touching = []
        else:
            touching = self.world.contact_bodies
        obs.extend([
            1.0 if self.world.bodies["Rulna"] in touching else 0.0,
            1.0 if self.world.bodies["Lulna"] in touching else 0.0,
            1.0 if self.world.bodies["Rfoot"] in touching else 0.0,
            1.0 if self.world.bodies["Lfoot"] in touching else 0.0,
        ])
        return torch.tensor(obs, dtype=torch.float32)
def rollout(actor, critic, env, steps):
    obs = env.reset()
    obs_buf, act_buf, logp_buf, val_buf, rew_buf, done_buf = [], [], [], [], [], []
    for t in range(steps):
        with torch.no_grad():
            d = actor.dist_builder(obs)
            action = d.sample()
            logp = d.log_prob(action).sum()
            value = critic(obs)
        next_obs, reward, done = env.step(action)
        obs_buf.append(obs); act_buf.append(action); logp_buf.append(logp)
        val_buf.append(value); rew_buf.append(reward); done_buf.append(done)
        obs = env.reset() if done else next_obs
    return obs_buf, act_buf, logp_buf, val_buf, rew_buf, done_buf, obs  # last obs = bootstrap
def gae(rewards, values, dones, bootstrap_value, gamma=0.99, lam = 0.95):
    N = len(rewards)
    advantages = [0.0]* N
    last_adv = 0.0
    for t in reversed(range(N)):
        next_value = bootstrap_value if t == N-1 else values[t+1]
        delta = rewards[t] + gamma*next_value*(1-dones[t])-values[t]
        last_adv = delta+gamma*lam*(1-dones[t])*last_adv
        advantages[t] = last_adv
    returns = [advantages[t]+values[t] for t in range(N)]
    return advantages, returns
def update(actor, critic, opt, obs_buf, act_buf, logp_buf, advantages, returns,
           epochs=10, clip=0.2, ent_coef=0.01, vf_coef=0.5):
    obs_t    = torch.stack(obs_buf)                        # (N, 23)
    act_t    = torch.stack(act_buf)                        # (N, 16)
    logp_old = torch.stack(logp_buf).detach()             # (N,)
    adv_t    = torch.tensor(advantages, dtype=torch.float32)
    ret_t    = torch.tensor(returns, dtype=torch.float32)
    adv_t    = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)

    params = list(actor.parameters()) + list(critic.parameters())

    for epoch in range(epochs):
        d        = actor.dist_builder(obs_t)
        new_logp = d.log_prob(act_t).sum(-1)              # (N,)
        entropy  = d.entropy().sum(-1).mean()
        new_val  = critic(obs_t).squeeze(-1)             # (N,)

        ratio = torch.exp(new_logp - logp_old)
        if epoch == 0:
            assert torch.allclose(ratio, torch.ones_like(ratio), atol=1e-4)

        clipped     = torch.clamp(ratio, 1 - clip, 1 + clip)
        policy_loss = -torch.min(ratio * adv_t, clipped * adv_t).mean()
        value_loss  = ((new_val - ret_t) ** 2).mean()
        loss        = policy_loss + vf_coef * value_loss - ent_coef * entropy

        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, 0.5)
        opt.step()
def main():
    actor = Actor(27, 64, 20)
    critic = Critic(27, 64)
    opt = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=3e-4)
    env = Env()
    log = open("rewards.log", "a")
    for it in range(2000):
        obs_buf, act_buf, logp_buf, val_buf, rew_buf, done_buf, last_obs = rollout(actor, critic, env, 2048)
        values = [v.item() for v in val_buf]
        with torch.no_grad():
            bootstrap = critic(last_obs).item()
        advantages, returns = gae(rew_buf, values, done_buf, bootstrap)

        update(actor, critic, opt, obs_buf, act_buf, logp_buf, advantages, returns)

        ep_reward = sum(rew_buf)
        msg = f"iter {it:4d}  reward-sum {ep_reward:8.2f}"
        print(msg, flush=True)
        log.write(msg + "\n"); log.flush()

        if it % 10 == 0:
            torch.save(actor.state_dict(), "actor.pt")
            torch.save(critic.state_dict(), "critic.pt")
    log.close()
if __name__ == "__main__":
    main()