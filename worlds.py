import itertools
import math

class World:
    def __init__(self, FPS, height, width, k=30000, e=0.5, c=25, mu=0.8, solver = "penalty", joint_beta = 0.5, iters = 10):
        self.dt = 1/FPS
        self.height = height
        self.solver = solver
        self.k = k
        self.c = c
        self.e = e
        self.width = width
        self.mu = mu
        self.iters = iters
        self.joint_beta = joint_beta
        self.no_collide = set()
        self.bodies = {}
        self.springs = {}
        self.joints = {}
        self.bodies["floor"] = Body(x=0, y = self.height-2, y_vel = 0, x_vel=0, mass=0, fx=0, fy=0, k=1e12, c=1e12, static=True, shape=Plane(0, -1), )



    def solve_contacts(self):
        contacts = []
        for a, b in itertools.combinations(self.bodies.values(), 2):
            if a.group is not None and a.group == b.group:
                continue
            if a.inv_mass + b.inv_mass == 0:
                continue
            if frozenset((a,b)) in self.no_collide:
                continue
            for c in contact(a, b):
                contacts.append(c)
        for a, b, nx, ny, p, px, py in contacts:   
            beta, slop = 0.2, 0.005
            corr = max(p - slop, 0) * beta / (a.inv_mass + b.inv_mass)
            a.x -= corr * a.inv_mass * nx
            a.y -= corr * a.inv_mass * ny
            b.x += corr * b.inv_mass * nx
            b.y += corr * b.inv_mass * ny
        for jnt in self.joints.values():
            jnt.correct_position(self.joint_beta)

        for _ in range(self.iters):
                for c in contacts:
                    a,b, nx, ny, p, px, py=c
                    rax, ray = px-a.x, py-a.y
                    rbx, rby = px-b.x, py-b.y
                    svx = (b.x_vel - b.omega*rby) - (a.x_vel - a.omega*ray) 
                    svy = (b.y_vel + b.omega*rbx) - (a.y_vel + a.omega*rax)
                    v_n = svx*nx + svy*ny
                    if v_n > 0: continue
                    e = min(a.e, b.e)
                    if v_n > -0.06:
                        e = 0
                    racn = rax*ny - ray*nx            
                    rbcn = rbx*ny - rby*nx
                    m_n = 1/(a.inv_mass + b.inv_mass + racn*racn*a.inv_I + rbcn*rbcn*b.inv_I)
                    j = -(1 + e) * v_n * m_n
                    b.app_impulse( j*nx,  j*ny, px, py)
                    a.app_impulse(-j*nx, -j*ny, px, py)
                    svx = (b.x_vel - b.omega*rby) - (a.x_vel - a.omega*ray)
                    svy = (b.y_vel + b.omega*rbx) - (a.y_vel + a.omega*rax)

                    tx, ty = -ny, nx                    # tangent: the normal rotated 90°
                    v_t = svx*tx + svy*ty               # slip: how fast the skins scrub sideways

                    ract = rax*ty - ray*tx              # leverage of this contact to spin a
                    rbct = rbx*ty - rby*tx              # ...and b
                    m_t = 1/(a.inv_mass + b.inv_mass + ract*ract*a.inv_I + rbct*rbct*b.inv_I)

                    jt = -m_t * v_t                     # the exact impulse that makes slip = 0

                    mu_eff = (a.mu*b.mu)**0.5           # same pair rule as penalty
                    jt = max(-mu_eff*j, min(mu_eff*j, jt))   # Coulomb budget: |jt| ≤ μ·j

                    b.app_impulse( jt*tx,  jt*ty, px, py)
                    a.app_impulse(-jt*tx, -jt*ty, px, py) 
                for jnt in self.joints.values():
                    jnt.solve()
    def create_spring(self, links, stretch_res, length, name1, name2, spring_name, mass=1.0):
        if links == 0:
            self.springs[spring_name] = Spring(links = links, stretch_res = stretch_res, length = length, body1 = self.bodies[name1], body2= self.bodies[name2])
        elif links >0:
            link = self.create_chain(links, stretch_res, length, self.bodies[name1], self.bodies[name2], spring_name, mass)
            for i, s in enumerate(link):
                self.springs[f"{spring_name}{i}"] = s
    def find_interact(self):
        for a, b in itertools.combinations(self.bodies.values(), 2):
            if a.group is not None and a.group == b.group:
                continue
            if a.inv_mass + b.inv_mass == 0:
                continue
            for c in contact(a, b):        # a pair can yield 1 contact (usual) or 2 (flat capsule)
                self.resolve(*c)

    def app_N(self, a, b, nx, ny, N, px, py):
        b.app_force(N*nx, N*ny, px, py)
        a.app_force(-N*nx, -N*ny, px, py)
    def app_friction(self, a, b, nx, ny, rvx, rvy, N, mu_eff, m_red, px, py):
        tx, ty = -ny, nx
        rax, ray = px - a.x, py - a.y
        rbx, rby = px - b.x, py - b.y
        svx = (b.x_vel - b.omega*rby) - (a.x_vel - a.omega*ray)
        svy = (b.y_vel + b.omega*rbx) - (a.y_vel + a.omega*rax)
        t_along = tx*svx + ty*svy
        friction = mu_eff*N
        ract = rax*ty - ray*tx
        rbct = rbx*ty - rby*tx
        m_t = 1/(a.inv_mass + b.inv_mass + ract*ract*a.inv_I + rbct*rbct*b.inv_I)
        friction = min(friction, m_t*abs(t_along)/self.dt)        
        if t_along > 0:
            friction = -friction
        b.app_force(friction*tx, friction*ty, px, py)
        a.app_force(-friction*tx, -friction*ty, px, py)
    def resolve(self, a, b, nx, ny, p, px, py):
        rvx, rvy = b.x_vel-a.x_vel, b.y_vel-a.y_vel
        keff = (a.k*b.k)/(a.k+b.k)
        ceff = (a.c*b.c)/(a.c+b.c)
        mu_eff = (a.mu*b.mu)**0.5
        m_red = 1/(a.inv_mass + b.inv_mass)
        v_along = rvx*nx + rvy*ny
        N = max(0, keff*p - ceff*v_along)
        self.app_N(a, b, nx, ny, N, px, py)
        self.app_friction(a, b, nx, ny, rvx, rvy, N, mu_eff, m_red, px, py)
    def add_spring(self, body, name):
        self.springs[name] = body
    def add_body(self, body, name):
        self.bodies[name] = body
    def add_joint(self, joint, name):
        self.joints[name] = joint
        self.no_collide.add(frozenset((joint.body1, joint.body2)))
    def step(self):
        for body in self.bodies.values():
            body.fx, body.fy, body.torque = 0, 0, 0
            body.app_force(0, body.mass*9.81)
        for spring in self.springs.values():
            spring.app_force()
        if self.solver == "penalty":
            self.find_interact()
        for body in self.bodies.values():
            body.app_acel(self.dt)
        if self.solver == "impulse":
            self.solve_contacts()
        for body in self.bodies.values():
            body.app_vel(self.dt)
    def create_chain(self, links, stretch_res, length,  body1, body2, spring_name, mass):
        lnk = []
        lnk.insert(0, body1)
        for i in range(1, links):
            t = i/links
            x = body1.x + t * (body2.x - body1.x)
            y = body1.y + t * (body2.y - body1.y)
            self.bodies[f"link{i}{spring_name}"] = Body(x, y, 0, 0, mass/links, group="link", shape=Disk(0.05))
            lnk.append(self.bodies[f"link{i}{spring_name}"])
        lnk.append(body2)
        link = [Spring(0, stretch_res, length/links, lnk[s-1], lnk[s]) for s in range(1, len(lnk))]
        return link        
    def add_rope(self, x1, y1, x2, y2, links, radius, mass, name):
        span = math.dist((x2,y2),(x1,y1))
        rope = []
        dy, dx = (y2-y1), (x2-x1)
        dy_step, dx_step = dy/links, dx/links
        for i in range(links):
            cx, cy = x1 + dx_step*(i+0.5), y1 + dy_step*(i+0.5)
            seg = Body(cx, cy, mass = mass/links, group=None, theta = math.atan2(dy,dx), shape=Capsule(span/(2*links), radius))
            self.bodies[f"{name}{i}"] = seg
            rope.append(seg)
            if i != 0:
                segjnt = Joint(rope[i-1], rope[i], (span/(2*links), 0), (-span/(2*links),0) )
                self.add_joint(segjnt, f"{name}{i}jnt")   # through add_joint so no_collide learns the pair       
        return rope

class Body:
    def __init__(self, x, y, x_vel=0, y_vel=0, mass=1, fx=0, fy=0, group = None, k=30000, e=0.5,c=25, mu=0.5, theta=0, omega=0, torque=0, I=None, shape=None, static=False):
        self.x = x
        self.y = y
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.mass = mass
        self.fx = fx
        self.fy=fy
        self.e = e
        self.group = group
        self.k = k
        self.c = c
        self.mu = mu
        self.theta = theta
        self.omega = omega
        self.torque = torque
        if shape is None:
            self.shape = Disk(1)
        else:
            self.shape = shape
        if static:
            self.I = 0
            self.inv_mass, self.inv_I = 0, 0
            self.k = 1e12
            self.c = 1e12
            mu=0.8
            mass=1
        else:
            if I is None:
                I = self.mass * self.shape.moi_per_mass()
            self.I = I
            self.inv_mass, self.inv_I = 1/self.mass, 1/self.I
    def app_force(self, fx, fy, px= None, py= None):
        self.fx += fx
        self.fy += fy
        if px is not None:
            rx, ry = -self.x+px, -self.y+py
            self.torque += rx*fy - ry*fx
    def app_acel(self, dt):
        self.y_vel += self.fy*self.inv_mass*dt
        self.x_vel += self.fx*self.inv_mass*dt
        self.omega += self.torque * self.inv_I * dt
    def app_vel(self, dt):
        self.x += self.x_vel*dt
        self.y += self.y_vel*dt
        self.theta += self.omega * dt
    def app_impulse(self, jx, jy, px=None, py=None):
        self.x_vel +=jx*self.inv_mass
        self.y_vel += jy*self.inv_mass
        if px is not None:
            rx, ry = -self.x+px, -self.y+py
            self.omega += (rx*jy - ry*jx)*self.inv_I
    def _world_point(self, ox, oy):
        c, s = math.cos(self.theta), math.sin(self.theta)
        px = self.x + ox*c - oy*s
        py = self.y + ox*s + oy*c
        return px, py
    @property
    def radius(self):
        return self.shape.radius
    
    @property
    def area(self):
        return self.shape.area()

class Joint:
    def __init__(self, body1, body2, off1, off2):
        self.off1 = off1
        self.off2 = off2
        self.body1 = body1
        self.body2 = body2
    def solve(self):
        b1, b2 = self.body1, self.body2
        p1x, p1y = b1._world_point(*self.off1)
        p2x, p2y = b2._world_point(*self.off2)
        rax, ray = p1x - b1.x, p1y - b1.y
        rbx, rby = p2x - b2.x, p2y-b2.y
        vx = (b2.x_vel - b2.omega*rby) - (b1.x_vel - b1.omega*ray)   
        vy = (b2.y_vel + b2.omega*rbx) - (b1.y_vel + b1.omega*rax)
        ima, imb, ia, ib = b1.inv_mass, b2.inv_mass, b1.inv_I, b2.inv_I
        k00 = ima + imb + ia*ray*ray + ib*rby*rby
        k01 = -ia*rax*ray - ib*rbx*rby
        k11 = ima + imb + ia*rax*rax + ib*rbx*rbx
        det = k00*k11 - k01*k01
        jx = (-k11*vx + k01*vy) / det         
        jy = ( k01*vx - k00*vy) / det
        b2.app_impulse( jx,  jy, p2x, p2y)     # each body kicked at its own anchor
        b1.app_impulse(-jx, -jy, p1x, p1y)
    def correct_position(self, beta):
        b1, b2 = self.body1, self.body2
        p1x, p1y = b1._world_point(*self.off1)
        p2x, p2y = b2._world_point(*self.off2)
        dx, dy = p2x - p1x, p2y - p1y           
        total = b1.inv_mass + b2.inv_mass
        if total == 0: return               
        b1.x += dx * beta * b1.inv_mass / total   
        b1.y += dy * beta * b1.inv_mass / total
        b2.x -= dx * beta * b2.inv_mass / total   
        b2.y -= dy * beta * b2.inv_mass / total
            





class Spring:
    def __init__(self, links, stretch_res, length,  body1, body2, c_spring = 2):
        self.body1 = body1
        self.body2 = body2
        self.res = stretch_res
        self.links = links
        if self.links > 0:
            self.create_chain()
        self.len = length
        self.c = c_spring


    def app_force(self):
        dist = math.dist((self.body1.x, self.body1.y), (self.body2.x,self.body2.y))
        stretch = dist - self.len
        dx = self.body2.x - self.body1.x
        dy = self.body2.y - self.body1.y
        nx, ny = dx/dist, dy/dist
        rvx= self.body2.x_vel - self.body1.x_vel
        rvy= self.body2.y_vel - self.body1.y_vel
        v_along = rvx*nx + rvy*ny
        force = self.res*stretch + self.c*v_along
        self.body1.app_force(force*nx, force*ny)
        self.body2.app_force(-force*nx, -force*ny)
class Limit:
    def __init__(self, body1, body2, min_ang, max_ang):
        self.body1 = body1
        self.body2 = body2
        self.b1 = self.body1     # short aliases; body1/body2 is the constraint contract add_joint relies on
        self.b2 = self.body2
        self.min = min_ang
        self.max = max_ang
    def solve(self):
        ang = self.b2.theta - self.b1.theta
        if self.min <= ang <= self.max:
            return
        w = self.b2.omega - self.b1.omega
        if ang > self.max and w <= 0:
            return
        if ang<self.min and w >= 0:
            return
        denom = self.b1.inv_I+self.b2.inv_I
        if denom == 0:
            return
        j = -w/denom
        self.b2.omega += j * self.b2.inv_I
        self.b1.omega -= j*self.b1.inv_I
    def correct_position(self, beta):
        ang = self.b2.theta - self.b1.theta
        over = 0
        if ang > self.max: over = ang - self.max
        elif ang< self.min: over = ang-self.min
        if over == 0:
            return
        denom = self.b1.inv_I + self.b2.inv_I
        if denom == 0:
            return
        self.b2.theta -= over * beta * self.b2.inv_I/ denom
        self.b1.theta += over*beta*self.b1.inv_I/denom
class Plane():
    def __init__(self, nx, ny):
        self.nx = nx/math.hypot(nx,ny)
        self.ny=ny/math.hypot(nx,ny)
class Disk():
    def __init__(self, radius):
        self.radius = radius
    def area(self):
        return math.pi*self.radius**2
    def moi_per_mass(self):
        return self.radius**2/2
class Capsule:
    def __init__(self, half_length, radius):
        self.half_length = half_length
        self.radius = radius
    def area(self):
        return 4*self.half_length*self.radius + math.pi*self.radius**2
    def moi_per_mass(self):
        return (self.half_length**2 + self.radius**2)/3     # rectangle approx (exact at r=0: rod)
    def endpoints(self, body):
        c, s = math.cos(body.theta), math.sin(body.theta)
        hx, hy = self.half_length*c, self.half_length*s
        return (body.x - hx, body.y - hy), (body.x + hx, body.y + hy)


def closest_on_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    denom = abx*abx + aby*aby
    if denom == 0:
        return ax, ay                       # degenerate zero-length segment
    t = ((px - ax)*abx + (py - ay)*aby) / denom
    t = max(0.0, min(1.0, t))               # clamp to stay ON the segment (handles round caps)
    return ax + t*abx, ay + t*aby


def capsule_disk(cap, disk):
    A, B = cap.shape.endpoints(cap)
    cx, cy = closest_on_segment(disk.x, disk.y, A[0], A[1], B[0], B[1])
    dx, dy = disk.x - cx, disk.y - cy        # spine point -> disk center = a -> b
    dist = math.hypot(dx, dy)
    p = (cap.radius + disk.radius) - dist
    if p <= 0 or dist == 0:
        return []
    nx, ny = dx/dist, dy/dist
    px, py = cx + nx*cap.radius, cy + ny*cap.radius   # contact on capsule's skin
    return [(cap, disk, nx, ny, p, px, py)]
def capsule_plane(plane, cap):
    nx, ny = plane.shape.nx, plane.shape.ny
    A, B = cap.shape.endpoints(cap)
    out = []
    for ex, ey in (A, B):                             # BOTH end-caps -> up to 2 contacts
        d = (ex - plane.x)*nx + (ey - plane.y)*ny     # height of endpoint above surface
        p = cap.radius - d
        if p > 0:
            px, py = ex - nx*cap.radius, ey - ny*cap.radius
            out.append((plane, cap, nx, ny, p, px, py))
    return out

def capsule_capsule(ca, cb):
    A0, A1 = ca.shape.endpoints(ca)
    B0, B1 = cb.shape.endpoints(cb)
    cands = [
        (A0, closest_on_segment(*A0, *B0, *B1)),
        (A1, closest_on_segment(*A1, *B0, *B1)),
        (closest_on_segment(*B0, *A0, *A1), B0),
        (closest_on_segment(*B1, *A0, *A1), B1),
    ]
    pa, pb = min(cands, key=lambda pr: math.dist(pr[0], pr[1]))
    dx, dy = pb[0]-pa[0], pb[1]-pa[1]
    dist = math.hypot(dx, dy)
    p = (ca.radius + cb.radius) - dist
    if p <= 0 or dist == 0:
        return []
    nx, ny = dx/dist, dy/dist
    px, py = pa[0] + nx*ca.radius, pa[1] + ny*ca.radius
    return [(ca, cb, nx, ny, p, px, py)]


def disk_disk(a, b):
    dx = b.x - a.x
    dy = b.y - a.y
    dist = math.dist((a.x, a.y), (b.x, b.y))
    p = (a.radius + b.radius) - dist
    if p <= 0 or dist == 0:
        return []
    nx = dx/dist
    ny = dy/dist
    px = a.x + a.radius*nx
    py = a.y + a.radius*ny
    return [(a, b, nx, ny, p, px, py)]
def disk_plane(plane,disk):
    nx, ny = plane.shape.nx, plane.shape.ny
    d = (disk.x - plane.x)*nx + (disk.y-plane.y)*ny
    p = disk.radius-d
    px, py = disk.x-disk.radius*nx, disk.y-disk.radius*ny
    if p <= 0:
        return []
    return [(plane, disk, nx, ny, p, px, py)]




SHAPE_ORDER = {Plane: 0, Capsule: 1, Disk: 2}
PAIRS = {(Plane, Disk):      disk_plane,
         (Disk, Disk):       disk_disk,
         (Plane, Capsule):   capsule_plane,
         (Capsule, Disk):    capsule_disk,
         (Capsule, Capsule): capsule_capsule}

def contact(a,b):
    a_rank = SHAPE_ORDER[type(a.shape)]
    b_rank = SHAPE_ORDER[type(b.shape)]
    rank_order = (a,b) if a_rank <= b_rank else (b,a)
    a,b = rank_order
    fn = PAIRS.get((type(a.shape), type(b.shape)))
    if fn is None:
        return []
    return fn(a, b)
    

