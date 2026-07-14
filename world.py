import itertools
import math

class World:
    def __init__(self, FPS, height, width, k=300, c=2.5, mu=0.8):
        self.dt = 1/FPS
        self.height = height
        self.k = k
        self.c = c
        self.width = width
        self.mu = mu
        self.bodies = {}
        self.springs = {}
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
            if math.dist((a.x, a.y), (b.x, b.y)) < a.radius +b.radius:
                self.ball_interact(a, b)
    def app_friction(self,a,b, p):
        rvx, rvy = b.x_vel-a.x_vel, b.y_vel-a.y_vel
        keff = (a.k*b.k)/(a.k+b.k)
        ceff = (a.c*b.c)/(a.c+b.c)
        mu_eff = (a.mu*b.mu)**(1/2)
        dx = b.x - a.x
        dy = b.y - a.y
        dist = math.dist((a.x, a.y), (b.x,b.y))
        nx = dx/dist
        ny = dy/dist
        v_along = (rvx*nx + rvy*ny)
        force = max(0, keff*p -ceff*v_along)
        tx, ty = -ny, nx
        t_along = tx*rvx+ ty*rvy
        friction = mu_eff*force
        m_red = 1/(a.inv_mass + b.inv_mass)
        friction = min(friction, m_red*abs(t_along)/self.dt)
        if t_along > 0:
            friction = -friction
        b.app_force(friction*tx, friction*ty)
        a.app_force(-friction*tx, -friction*ty)

    def ball_interact(self, a, b):
        rvx, rvy = b.x_vel-a.x_vel, b.y_vel-a.y_vel
        keff = (a.k*b.k)/(a.k+b.k)
        ceff = (a.c*b.c)/(a.c+b.c)
        dx = b.x - a.x
        dy = b.y - a.y
        dist = math.dist((a.x, a.y), (b.x,b.y))
        p = (a.radius+b.radius) - dist
        nx = dx/dist
        ny = dy/dist
        v_along = (rvx*nx + rvy*ny)
        force = max(0, keff*p -ceff*v_along)
        b.app_force(force*nx, force*ny)
        a.app_force(-force*nx, -force*ny)
        self.app_friction(a, b, p)
    def integrate_f(self):
        for body in self.bodies.values():
            body.app_acel(body.fx, body.fy, self.dt)
            body.app_vel(self.dt)
        for ball in self.bodies.values():
            if ball.y + ball.radius > self.height:
                self.floor(ball)

    def add_body(self, body, name):
        self.bodies[name] = body
    def step(self):
        for body in self.bodies.values():
            body.fx, body.fy = 0, 0
            body.app_force(0, body.mass*9.81)
        for spring in self.springs.values():
            spring.app_force()
        self.find_interact()
        self.integrate_f()
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
    def floor(self, body):
            if body.group == "link":
                pass 
            else:
                p = body.y+body.radius - self.height
                c_s = body.y_vel
                N = min(0, -body.k*p - body.c*c_s)
                mu_eff = (body.mu*self.mu)**0.5
                friction = min(mu_eff*abs(N), body.mass*abs(body.x_vel)/self.dt)
                if body.x_vel > 0:
                    friction = -friction
                body.app_acel(x_force = friction, y_force = N, dt=self.dt)

class Body:
    def __init__(self, x, y, x_vel, y_vel, mass, fx=0, fy=0, group = None, k=300, c=2.5, mu=0.5, theta=0, omega=0, torque=0, I=None, shape=None, static=False):
        self.x = x
        self.y = y
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.mass = mass
        self.fx = fx
        self.fy=fy
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
        else:
            if I is None:
                I = self.mass * self.shape.moi_per_mass()
            self.I = I
            self.inv_mass, self.inv_I = 1/self.mass, 1/self.I
    def app_force(self, fx, fy):
        self.fx += fx
        self.fy += fy
    def app_acel(self, x_force, y_force, dt):
        self.y_vel += y_force*self.inv_mass*dt
        self.x_vel += x_force*self.inv_mass*dt
    def app_vel(self, dt):
        self.x += self.x_vel*dt
        self.y += self.y_vel*dt
    @property
    def radius(self):
        return self.shape.radius
    
    @property
    def area(self):
        return self.shape.area()



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

