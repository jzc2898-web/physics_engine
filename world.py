import itertools
import math

class World:
    def __init__(self, FPS, height, width, k=300, c=2.5):
        self.dt = 1/FPS
        self.height = height
        self.k = k
        self.c = c
        self.width = width
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
    def ball_interact(self, a, b):
        dx = b.x - a.x
        dy = b.y - a.y
        dist = math.dist((a.x, a.y), (b.x,b.y))
        p = (a.radius+b.radius) - dist
        nx = dx/dist
        ny = dy/dist
        force = self.k*p 
        b.app_force(force*nx, force*ny)
        a.app_force(-force*nx, -force*ny)
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
            self.bodies[f"link{i}{spring_name}"] = Body(x, y, 0, 0, mass/links, 0, 0, 0.05, "link")
            lnk.append(self.bodies[f"link{i}{spring_name}"])
        lnk.append(body2)
        link = [Spring(0, stretch_res, length/links, lnk[s-1], lnk[s]) for s in range(1, len(lnk))]
        return link
    def floor(self, body):
            if body.group is "link":
                pass 
            else:
                p = body.y+body.radius - self.height
                c_s = body.y_vel
                body.app_acel(x_force = 0, y_force = min(0,-self.k*p-self.c*c_s), dt=self.dt)
class Body:
    def __init__(self, x, y, x_vel, y_vel, mass, fx=0, fy=0, radius=1, group = None):
        self.x = x
        self.y = y
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.mass = mass
        self.fx = fx
        self.fy=fy
        self.radius = radius
        self.group = group
    def app_force(self, fx, fy):
        self.fx += fx
        self.fy += fy
    def app_acel(self, x_force, y_force, dt):
        self.y_vel += y_force/self.mass*dt
        self.x_vel += x_force/self.mass*dt
    def app_vel(self, dt):
        self.x += self.x_vel*dt
        self.y += self.y_vel*dt

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


