from ppo import gae

adv, ret = gae(
    rewards = [1,1,1,1],
    values = [0,0,0,0],
    dones = [0,0,0,0],
    bootstrap_value = 0,
    gamma=1, lam=1
)
print(adv)