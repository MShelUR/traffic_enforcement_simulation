# built-ins
from math import radians, cos, sin
import tkinter as tk
import time

# external libraries
from shapely.geometry import Polygon

############################
# tkinter setup
############################

start_time = time.perf_counter()

root = tk.Tk()
root.title("Car Simulation")

canvas = tk.Canvas(root, width=1440, height=1080, bg="white")
canvas.pack(fill=tk.BOTH, expand=True)

############################
# simulation utilities
############################

def rotate_polygon(points, deg_angle, center):
    angle = radians(deg_angle)
    cos_angle = cos(angle)
    sin_angle = sin(angle)
    cx, cy = center
    new_points = []
    #print(points)
    for x_old, y_old in points:
        # get relative position from old point to center
        x_old -= cx
        y_old -= cy

        # find rotated offset
        dx = x_old * cos_angle - y_old * sin_angle
        dy = x_old * sin_angle + y_old * cos_angle

        # add that offset to the center, that is the new point
        new_points.append((cx + dx, cy + dy))

    return new_points

def get_points(pos,size,rot):
    return rotate_polygon(
        [
            (pos[0]+size[0], pos[1]+size[1]),
            (pos[0]+size[0], pos[1]-size[1]),
            (pos[0]-size[0], pos[1]-size[1]),
            (pos[0]-size[0], pos[1]+size[1])
        ],
        rot,
        (pos[0], pos[1])
    )

def is_intersecting(obj1,obj2):
    return Polygon(obj1.points).intersects(Polygon(obj2.points))

def magnitude(vector):
    return (vector[0]**2+vector[1]**2)**.5

############################
# the car
############################

class Car:
    global canvas
    count = 0

    def __init__(self,size,pos,rot):
        Car.count += 1
        self.id = "car_"+str(Car.count)

        self.size = list(size)
        self.position = list(pos)
        self.rot = rot

        self.center = (self.position[0]+self.size[0]/2, self.position[1]+self.size[1]/2)

        # apply initial rotation
        self.points = get_points(pos,size,rot)

        self.tk_id = canvas.create_polygon(self.points, fill="black", outline="black", tags=self.id)

        self.velocity = [0,0]
        self.acceleration = [0,0]
        self.steering_direction = 0

    def accelerate(self,dx,dy):
        self.acceleration[0] = dx
        self.acceleration[1] = dy

    def steer(self, angle):
        self.steering_direction = angle

    def turn():
        pass

    def step(self):
        # step vehicle

        ## apply rotation to acceleration
        angle = radians(self.rot)
        cos_angle = cos(angle)
        sin_angle = sin(angle)
        accel_x = self.acceleration[0]
        accel_y = self.acceleration[1]

        local_acceleration = [
            accel_x * cos_angle - accel_y * sin_angle,
            accel_x * sin_angle + accel_y * cos_angle
        ]

        # update velocity and position with projected orientation
        self.velocity[0] += local_acceleration[0]
        self.velocity[1] += local_acceleration[1]
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]

        steer = self.steering_direction*magnitude(self.velocity)
        self.rot += steer

        # apply drag
        self.velocity[0] *= .9
        self.velocity[1] *= .9
        self.acceleration[0] *= .8
        self.acceleration[1] *= .8
        self.steering_direction *= .5

        # update points in polygon
        self.points = get_points(self.position,self.size,self.rot)


        # redraw polygon
        canvas.coords(self.id, self.points)

class Wall:
    global canvas
    count = 0

    def __init__(self,points):
        Wall.count += 1
        self.id = "wall"+str(Wall.count)
        self.points = points
        self.tk_id = canvas.create_polygon(points, fill="white", outline="black", tags=self.id)

############################
# input handling
############################

# keys

keys_held = {
    "w": False, 
    "a": False,
    "s": False,
    "d": False
    }

def on_key_press(event):
    key = event.char
    if key in keys_held:
        keys_held[key] = True
    
def on_key_release(event):
    key = event.char
    if key in keys_held:
        keys_held[key] = False

root.bind("<KeyPress>", on_key_press)
root.bind("<KeyRelease>", on_key_release)


# mouse (wall drawing)

wall_points = []
temp_lines = []

def on_mouse_click(event):
    global wall_points
    global temp_lines
    if event.num == 1: # left click
        wall_points.append((event.x,event.y))
        if len(wall_points) > 1:
            temp_lines.append(canvas.create_line(wall_points[-2],wall_points[-1]))

        d_vector = (event.x-wall_points[0][0], event.y-wall_points[0][1]) # dist between latest and first point
        
        if magnitude(d_vector) < 10 and len(wall_points) >= 4:
            walls.append(Wall(wall_points))
            wall_points = []
            for line in temp_lines:
                canvas.delete(line)
            temp_lines = []


root.bind("<Button-1>", on_mouse_click)


############################
# basic simulation
############################

car1 = Car((18,14),(100,100),90)

wall1 = Wall([(200,40),(200,120),(220,120),(220,40)])

cars = [car1]
walls = [wall1]

ms = 10 # milliseconds for each step of the simulation

def main_loop():
    global start_time
    continue_simulation = True # sentinel value for game loop
    # move player controlled car
    car1.step()
    if keys_held["w"]:
        car1.accelerate(.5,0)
    if keys_held["s"]:
        car1.accelerate(-.5,0)
    if keys_held["a"]:
        car1.steer(-1)
    if keys_held["d"]:
        car1.steer(1)
    
    # check for collisions
    for car in cars:
        collided = False
        for wall in walls:
            if is_intersecting(car, wall):
                collided = True
        if collided:
            canvas.itemconfig(car.tk_id, fill="red")
            continue_simulation = False
        else:
            canvas.itemconfig(car.tk_id, fill="black")
    
    if continue_simulation:
        root.after(ms,main_loop)
        print(time.perf_counter()-start_time)
        start_time = time.perf_counter()
main_loop()


root.mainloop()