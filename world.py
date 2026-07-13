

class World:
    def __init__(self, FPS, height, width, k=300, c=2.5):
        self.dt = 1/FPS
        self.height = height
        self.k = k
        self.c = c
        self.width = width
    def app_grav(self, body):
        self.floor(body)
        body.app_acel(0, body.mass*9.81, self.dt)
        body.app_vel(self.dt)
    def floor(self, body):
        if body.y > self.height:
            p = body.y - self.height
            c_s = body.y_vel
            body.app_acel(x_force = 0, y_force = min(0,-self.k*p-self.c*c_s), dt=self.dt)
class Body:
    def __init__(self, x, y, x_vel, y_vel, mass):
        self.x = x
        self.y = y
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.mass = mass
    def app_acel(self, x_force, y_force, dt):
        self.y_vel += y_force/self.mass*dt
        self.x_vel += x_force/self.mass*dt
    def app_vel(self, dt):
        self.x += self.x_vel*dt
        self.y += self.y_vel*dt
