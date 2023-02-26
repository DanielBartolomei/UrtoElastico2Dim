import pygame as pyg
import math
import random
import numpy as np

# Colors
BKG = (199, 199, 199)
BALL1COLOR = (163, 44, 44)
BALL2COLOR = (35, 137, 176)
BALL3COLOR = (255, 153, 125)
BALL4COLOR = (209, 56, 0)
BALL5COLOR = (255, 204, 20)

# Window Setup
pyg.init()
WIDTH, HEIGHT =  800,800
WIN = pyg.display.set_mode((WIDTH, HEIGHT))
WIN.fill(BKG)
CLOCK = pyg.time.Clock()
FPS = 120
pyg.display.set_caption("Elastic Collision Simulation")

# Font Object
font = pyg.font.Font(None, 24)

# Set inital time and step
dt = 0.00833333333 # having 120 fps (1 / 120 = 0.00833333333...)

class Ball:
    def __init__(self, x, y, radius, color, mass, cdrag):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.mass = mass
        self.cdrag = cdrag
        self.trajectory = []
        self.v = [random.randrange(-1000,1000), random.randrange(-1000,1000)]
 
    def draw(self, win):
        pyg.draw.circle(win, self.color, (self.x,self.y), self.radius)

    def update_position(self):
        self.x += self.v[0] * dt
        self.y += self.v[1] * dt

    def check_border_collision(self):
        # Checks for ball 1 border collision
        if self.x - self.radius < 0 or self.x + self.radius > WIDTH: # right or left border
            self.v[0] = -self.v[0] # change x component
        if self.y - self.radius < 0 or self.y + self.radius > HEIGHT: # top or bottom border
            self.v[1] = -self.v[1] # change y component
    
    def check_ball_collision(self, balls):

        for ball in balls:
            if not ball == self:
                distance = math.sqrt((self.x - ball.x)**2 + (self.y - ball.y)**2)
                if distance <= self.radius + ball.radius :
                    # collision has happened
                    # calculate angle

                    angle = -math.atan2(ball.y - self.y, ball.x - self.x)

                    i = [(ball.x-self.x),(ball.y-self.y)]
                    i = i/np.linalg.norm(i) #Normalizzo il versore a causa del problema di sovrapposizione dei cerchi a causa di dt
                    j = [i[1],-i[0]]

                    self_v = self.v
                    ball_v = ball.v
                    c = self_v

                    self_v = np.add(np.inner(np.inner(self_v,j),j),np.inner(np.inner(ball_v,i),i))
                    ball_v = np.add(np.inner(np.inner(ball_v,j),j),np.inner(np.inner(c,i),i))

                    self.v = self_v
                    ball.v = ball_v

    def drag(self):
        g = 9.81 * 3779 * (dt) #1m = 3779 px e devo rapportare a dt
        at = -(self.cdrag * g) * dt
        v = math.sqrt(self.v[0]**2 + self.v[1]**2)
        if v>0:
            i = [self.v[0] / v, self.v[1] / v] # versore velocità
            self.v = np.add(self.v,np.inner(at,i)) #sommo V0 con at scomponendo tra asse x ed y con il versore i
            vf = math.sqrt(self.v[0] ** 2 + self.v[1] ** 2)
            if vf > v: # evitare errori di aprossimazione
                self.v = [0,0]

def main():

    b1 = Ball(WIDTH / 4, HEIGHT / 2, 26, BALL1COLOR, 1, 0)
    b2 = Ball(WIDTH / 2 , HEIGHT / 2, 30, BALL2COLOR, 1, 0)
    b3 = Ball(WIDTH / 4 * 3, HEIGHT / 2, 50, BALL3COLOR, 1, 0.2)
    b4 = Ball(WIDTH / 2 , HEIGHT / 4, 15, BALL4COLOR, 1, 0.5)

    balls = [b1, b2, b3, b4]

    # Main game loop
    running = True
    while running:
        WIN.fill(BKG)
        for event in pyg.event.get():
            if event.type == pyg.QUIT:
                running = False

        for ball in balls:
            ball.update_position()
            ball.check_border_collision()
            ball.check_ball_collision(balls)
            if ball.cdrag > 0: 
                ball.drag()
            ball.draw(WIN)


        print("{} {}".format(b3.v, b4.v))
            
        

        # Update display
        CLOCK.tick(FPS)
        pyg.display.update()


    pyg.quit()

main()

