"""Capsule playground: run a batch of capsule interactions and render each as a
ghosted 'timelapse' strip (faded early -> solid late) into one PNG.

Just a demo/inspection harness -- it imports the engine, never modifies it.
Run:  python capsule_playground.py   ->   capsule_timelapse.png
"""
import math
import matplotlib
matplotlib.use("Agg")                       # headless: write a file, no window
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

from world import World, Body, Disk, Capsule, Plane

FPS = 360

# ---------- drawing ----------------------------------------------------------

def cap_geom(x, y, theta, hl, r):
    """world endpoints + perpendicular for a capsule at (x,y,theta)."""
    c, s = math.cos(theta), math.sin(theta)
    ax, ay = x - hl*c, y - hl*s
    bx, by = x + hl*c, y + hl*s
    return (ax, ay), (bx, by), (-s, c)      # A, B, unit-perpendicular

def draw_snapshot(ax, rec, color, alpha):
    kind, x, y, theta, r, hl = rec
    if kind == "disk":
        ax.add_patch(Circle((x, y), r, color=color, alpha=alpha, lw=0))
        ax.plot([x, x + r*math.cos(theta)], [y, y + r*math.sin(theta)],
                color="white", alpha=alpha*0.85, lw=1.0)          # spin hand
    else:
        A, B, (px, py) = cap_geom(x, y, theta, hl, r)
        corners = [(A[0]+px*r, A[1]+py*r), (B[0]+px*r, B[1]+py*r),
                   (B[0]-px*r, B[1]-py*r), (A[0]-px*r, A[1]-py*r)]
        ax.add_patch(Polygon(corners, closed=True, color=color, alpha=alpha, lw=0))
        ax.add_patch(Circle(A, r, color=color, alpha=alpha, lw=0))
        ax.add_patch(Circle(B, r, color=color, alpha=alpha, lw=0))
        ax.plot([A[0], B[0]], [A[1], B[1]], color="white", alpha=alpha*0.7, lw=1.0)

def draw_plane(ax, body, xspan):
    nx, ny = body.shape.nx, body.shape.ny
    tx, ty = -ny, nx                        # tangent along the surface
    L = 200
    ax.plot([body.x - tx*L, body.x + tx*L], [body.y - ty*L, body.y + ty*L],
            color="0.35", lw=3, zorder=0)

# ---------- simulation -------------------------------------------------------

def record(body):
    if isinstance(body.shape, Capsule):
        return ("cap", body.x, body.y, body.theta, body.shape.radius, body.shape.half_length)
    return ("disk", body.x, body.y, body.theta, body.shape.radius, 0.0)

def run_scene(world, actors, seconds, n_snaps=6):
    """Step the world; capture a snapshot of each actor at n_snaps even times.
    actors: dict name -> color. Returns list of (name, color, [records over time])."""
    total = int(seconds*FPS)
    every = max(1, total // (n_snaps - 1))
    frames = {name: [] for name in actors}
    for i in range(total + 1):
        if i % every == 0:
            for name in actors:
                frames[name].append(record(world.bodies[name]))
        world.step()
    return [(name, actors[name], frames[name]) for name in actors]

# ---------- scenes -----------------------------------------------------------
# Each scene returns (world, actors{name:color}, seconds, title).
# Floor is auto-created by World at y = height-2.

STEEL, CRIMSON, GREEN, ORANGE = "#3b6ea5", "#c0392b", "#2e8b57", "#d98324"

def scene_hit_ball(spin):
    W = World(FPS, 12, 20)                                   # floor at y=10
    cap = Body(3, 9.6, x_vel=9, mass=2, mu=(0.35 if spin else 0.05), omega=(6 if spin else 0),
               shape=Capsule(1.4, 0.4))                      # grippier + backspin -> ball gets dragged/lifted
    ball = Body(10, 9.3, mass=1, mu=0.5, shape=Disk(0.7))
    W.add_body(cap, "cap"); W.add_body(ball, "ball")
    title = "Capsule slides into a ball " + ("WITH backspin" if spin else "(no spin)")
    return W, {"cap": STEEL, "ball": CRIMSON}, 2.0, title

def scene_floor():
    W = World(FPS, 12, 22)                                   # floor at y=10
    flat = Body(6, 7.4, theta=0.4, mass=2, shape=Capsule(1.5, 0.4))    # gentle drop, tilted
    upright = Body(16, 6.6, theta=1.40, mass=2, shape=Capsule(1.5, 0.4))  # ~10 deg off vertical -> topples
    W.add_body(flat, "flat"); W.add_body(upright, "up")
    return W, {"flat": STEEL, "up": GREEN}, 2.6, "Drop onto floor: tilted self-levels / near-upright topples"

def scene_ramp():
    W = World(FPS, 16, 26)
    th = math.radians(25)
    nx, ny = math.sin(th), -math.cos(th)                    # normal points up-out of the ramp
    ramp = Body(6, 6, static=True, mu=0.15, shape=Plane(nx, ny))   # slick enough to slide (mu_eff < tan25)
    W.add_body(ramp, "ramp")
    r = 0.4
    # lay the capsule flat on the ramp surface at the ramp anchor + one radius out
    tx, ty = -ny, nx
    cx, cy = 6 + nx*r, 6 + ny*r
    cap = Body(cx, cy, mass=2, mu=0.15, theta=math.atan2(ty, tx), shape=Capsule(1.5, r))
    W.add_body(cap, "cap")
    return W, {"cap": ORANGE}, 2.2, "Capsule released on a 25 degree (slick) ramp"

def scene_cap_cap():
    W = World(FPS, 12, 20)                                   # floor at y=10
    mover = Body(3, 9.6, x_vel=7, mass=3, mu=0.05, shape=Capsule(1.3, 0.4))
    # standing capsule rests on its lower cap: center = floor(10) - radius - half_length = 8.3
    target = Body(9, 8.3, theta=1.5708, mass=1, mu=0.2, shape=Capsule(1.3, 0.4))
    W.add_body(mover, "mover"); W.add_body(target, "target")
    return W, {"mover": STEEL, "target": CRIMSON}, 2.2, "Capsule broadsides a standing capsule"

# ---------- render -----------------------------------------------------------

def main():
    scenes = [scene_hit_ball(False), scene_hit_ball(True),
              scene_floor(), scene_ramp(), scene_cap_cap()]
    fig, axes = plt.subplots(len(scenes), 1, figsize=(13, 3.0*len(scenes)))
    for ax, (world, actors, seconds, title) in zip(axes, scenes):
        planes = [b for b in world.bodies.values() if isinstance(b.shape, Plane)]
        tracks = run_scene(world, actors, seconds)
        for pl in planes:
            draw_plane(ax, pl, world.width)
        n = len(next(iter(tracks))[2])
        for name, color, recs in tracks:
            for i, rec in enumerate(recs):
                alpha = 0.15 + 0.85*(i/(n-1))               # ghost -> solid
                draw_snapshot(ax, rec, color, alpha)
        ax.set_title(title, fontsize=11, loc="left")
        ax.set_aspect("equal"); ax.invert_yaxis()           # +y is down in the engine
        ax.set_xlim(0, world.width); ax.set_ylim(world.height, 0)
        ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle("Capsule playground  --  faded = earlier, solid = later", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig("capsule_timelapse.png", dpi=110)
    print("wrote capsule_timelapse.png")

if __name__ == "__main__":
    main()
