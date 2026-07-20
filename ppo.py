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
    def __init__(self, max_steps=1000):
        self.world_fps = 360             # physics rate (small dt so limbs can't tunnel)
        self.substeps = 6                # physics steps per control step -> control = 360/6 = 60 Hz
        self.control_hz = self.world_fps / self.substeps
        self.max_steps = max_steps
        self.contact_penalty = 0.03      # per forbidden body touching the ground, per step
        # only feet may touch (hands/ulnas are free: no penalty, no reward)
        self.forbidden = ["trunk", "Rfemur", "Lfemur", "Rtibia", "Ltibia", "Rhumerus", "Lhumerus"]
        self.handsfree_steps = int(2.0 * self.control_hz)   # hands off 2s -> positive reward x2
        self.min_height = 0.5            # trunk must be this far above the floor for travel to pay
        self.energy_cost = 0.002         # per unit of summed activation, per step: tension is never free
        self.jerk_cost = 0.005           # per unit of |action - last action|: thrashing costs
        self.reset()
    def step(self, action):
        acts = torch.clip(action, 0, 1).tolist()   # floats, not tensors -> physics stays in float math
        self.energy = sum(acts)
        self.jerk = sum(abs(a - p) for a, p in zip(acts, self.prev_acts))
        self.prev_acts = acts
        prev = self.world.bodies["trunk"].x
        for ac, mu in zip(acts, self.world.muscle_names):
            self.world.springs[mu].set_activation(ac)
        for _ in range(self.substeps):
            self.world.step()
        self.steps += 1
        obs = self.get_obs()
        rewards = self.get_reward(prev)
        if self.steps >= self.max_steps:
            return obs, rewards, True
        return obs, rewards, False
    def get_reward(self, prev):
        n_trunk = self.world.bodies["trunk"]
        dx = n_trunk.x - prev
        height = self.world.bodies["floor"].y - n_trunk.y   # y-down: floor minus trunk = elevation
        rewards = 0
        if height >= self.min_height:
            rewards += dx                # travel only pays when the body is up off the ground
        else:
            rewards -= abs(dx)           # sliding along while low actively costs
        rewards -= self.energy_cost * self.energy     # holding tension costs
        rewards -= self.jerk_cost * self.jerk         # changing it violently costs more
        for name in self.forbidden:                   # only feet may bear on the floor
            if self.world.bodies[name] in self.world.contact_bodies:
                rewards -= self.contact_penalty
        touching = self.world.contact_bodies
        if (self.world.bodies["Rulna"] in touching) or (self.world.bodies["Lulna"] in touching):
            self.hands_free = 0                       # a hand touched: streak resets
        else:
            self.hands_free += 1
        if self.hands_free > self.handsfree_steps and rewards > 0:
            rewards *= 2                              # sustained hands-off: positive reward doubled
        return rewards

    def reset(self):
        self.world = World(self.world_fps, 15, 15, solver = "impulse", iters = 10)
        self.world.bodies["floor"].mu = 1.0        # grippy ground: feet mu_eff ~1.4, dragged limbs ~0.7
        human = make_human(self.world)
        obs = self.get_obs(True)
        self.steps = 0
        self.hands_free = 0
        self.prev_acts = [0.0] * 20
        self.energy = 0.0
        self.jerk = 0.0
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
def _worker(conn, seed):
    # one persistent env per process: it lives across iterations so long episodes
    # get experienced end-to-end even though each collect is only ~2048/N steps.
    torch.set_num_threads(1)                 # 1 core each; no thread fighting
    torch.manual_seed(seed)
    env = Env()
    actor = Actor(27, 64, 20)
    critic = Critic(27, 64)
    obs = env.reset()
    while True:
        cmd, payload = conn.recv()
        if cmd == "close":
            conn.close()
            break
        a_sd, c_sd, steps = payload          # fresh weights every iteration (on-policy!)
        actor.load_state_dict(a_sd)
        critic.load_state_dict(c_sd)
        obs_buf, act_buf, logp_buf, val_buf, rew_buf, done_buf = [], [], [], [], [], []
        for _ in range(steps):
            with torch.no_grad():
                d = actor.dist_builder(obs)
                action = d.sample()
                logp = d.log_prob(action).sum()
                value = critic(obs)
            next_obs, reward, done = env.step(action)
            obs_buf.append(obs); act_buf.append(action); logp_buf.append(logp)
            val_buf.append(value); rew_buf.append(reward); done_buf.append(done)
            obs = env.reset() if done else next_obs
        conn.send(((obs_buf, act_buf, logp_buf, val_buf, rew_buf, done_buf), obs))

def main(workers=12, total_steps=2048):
    import multiprocessing as mp
    import time
    actor = Actor(27, 64, 20)
    critic = Critic(27, 64)
    opt = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=3e-4)

    conns, procs = [], []
    for i in range(workers):
        parent, child = mp.Pipe()
        p = mp.Process(target=_worker, args=(child, i), daemon=True)
        p.start()
        conns.append(parent); procs.append(p)
    per = total_steps // workers

    log = open("rewards.log", "a")
    for it in range(2000):
        t0 = time.perf_counter()
        payload = ("collect", (actor.state_dict(), critic.state_dict(), per))
        for c in conns:
            c.send(payload)

        obs_buf, act_buf, logp_buf, adv_all, ret_all = [], [], [], [], []
        ep_reward = 0.0
        for c in conns:
            (o, a, l, v, r, dn), last_obs = c.recv()
            values = [x.item() for x in v]
            with torch.no_grad():
                bootstrap = critic(last_obs).item()
            adv, ret = gae(r, values, dn, bootstrap)   # per-chunk: each worker is its own timeline
            obs_buf += o; act_buf += a; logp_buf += l
            adv_all += adv; ret_all += ret
            ep_reward += sum(r)

        update(actor, critic, opt, obs_buf, act_buf, logp_buf, adv_all, ret_all)

        sps = (per * workers) / (time.perf_counter() - t0)
        msg = f"iter {it:4d}  reward-sum {ep_reward:8.2f}  ({sps:5.0f} steps/s)"
        print(msg, flush=True)
        log.write(msg + "\n"); log.flush()

        if it % 10 == 0:
            torch.save(actor.state_dict(), "actor.pt")
            torch.save(critic.state_dict(), "critic.pt")
    log.close()
    for c in conns:
        c.send(("close", None))
if __name__ == "__main__":
    main()