# built-ins
from math import radians, cos, sin
from pathlib import Path
import tkinter as tk
import time

# external libraries
from shapely.geometry import Polygon, Point

############################
# tkinter setup
############################

start_time = time.perf_counter()

root = tk.Tk()
root.title("Car Simulation")

screen_width = 1440
screen_height = 1080

screen_center = (screen_width/2,screen_height/2)

canvas = tk.Canvas(root, width=screen_width, height=screen_height, bg="white")
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
        # [(int(point_x), int(point_y)) for point_x, point_y in self.points]
        canvas.coords(self.id, self.points)

############################
# pathfinding
############################


# set up pathfinding things
PATHFIND_DEBUG = True # draw pathfinding paths
nav_mesh = Polygon([
    (0,0),
    (screen_width,0),
    (screen_width,screen_height),
    (0,screen_height)
    ])

def get_point_distance(point,other):
    # 2d magnitude
    return ((point[0]-other[0])**2+(point[1]-other[1])**2)**.5

def compute_a_star(current_point,other_point,goal_point):
    # find relative value of a point

    utility = get_point_distance(other_point,goal_point)
    cost = get_point_distance(current_point,other_point)

    return utility / cost

def find_best_next_point(current_point,goal_point,others):
    best_point = None
    best_value = 0
    for other in others:
        new_value = compute_a_star(current_point,other,goal_point)
        if new_value > best_value:
            best_point = other
            best_value = new_value
    
    return best_point

def find_closest_point(point,others):
    closest_point = others[0]
    closest_dist = point_distance(point,closest_point)

    for other in others[1:]: # check the rest
        new_dist = point_distance(point,other)
        if new_dist < closest_dist:
            closest_dist = new_dist
            closest_point = other
    
    return closest_point

def pathfind_to_point(start,goal):
    nav_exterior = list(nav_mesh.exterior.coords)
    closest_start = find_closest_point(start,nav_exterior)
    closest_end = find_closest_point(goal,nav_exterior)

    path = [current_point]

    current_point = closest_start
    while current_point != closest_end:
        current_point = find_best_next_point(current_point,goal_point,nav_exterior)
        path.append(current_point)

    return path




############################
# wall handling
############################

walls = []


class Wall:
    global canvas
    global walls
    global nav_mesh
    count = 0

    def __init__(self,points):
        global nav_mesh
        Wall.count += 1
        self.id = "wall"+str(Wall.count)
        self.points = points
        self.tk_id = canvas.create_polygon(points, fill="white", outline="black", tags=self.id)
        nav_mesh = nav_mesh.difference(Polygon(points)) # take wall out of nav area
        walls.append(self)

wall_counter = 0
def save_wall_points(points):
    global wall_counter
    # convert points to line delimited x,y coordinates
    # save that as a new wall object
    str_output = "\n".join([str(point[0])+", "+str(point[1]) for point in points])
    with open(f"map/built/wall_{wall_counter}","w") as out_file:
        out_file.write(str_output)
    print(f"saved new wall to: map/built/wall_{wall_counter}")
    wall_counter += 1

def load_wall_from_file(wall_file):
    # load wall from file
    # update wall_counter if saving a new wall would cause an overwrite
    global wall_counter
    str_input = ""
    with open(wall_file,"r") as in_file:
        str_input = in_file.read()
    wall_input = [(float(point.split(", ")[0]),float(point.split(", ")[1])) for point in str_input.split("\n")]
    Wall(wall_input)

    if "map/built/wall_" in wall_file and wall_file[-1].isnumeric(): # update wall counter
        wall_counter = int(wall_file[-1])+1

def load_all_walls_from_map(map_path):
    # go through map folder recursively to load all walls
    remaining_paths = [map_path]
    while len(remaining_paths) > 0:
        new_paths = []
        for path in remaining_paths:
            for item in Path(path).iterdir():
                if item.is_file(): # is a geometry file, draw
                    load_wall_from_file(path+"/"+item.name)
                else: # is a path, append it
                    new_paths.append(path+"/"+item.name)
        remaining_paths = new_paths

load_all_walls_from_map("map")

############################
# procedural structures
############################

def make_roundabout(center, center_radius, road_width):

    # abbreviations so args are still descriptive
    c = center
    r = center_radius
    w = road_width

    # make the center of the roundabout
    Wall([
        (c[0]-r/2,c[1]-r/4),
        (c[0]-r/4,c[1]-r/2),
        (c[0]+r/4,c[1]-r/2),
        (c[0]+r/2,c[1]-r/4),
        (c[0]+r/2,c[1]+r/4),
        (c[0]+r/4,c[1]+r/2),
        (c[0]-r/4,c[1]+r/2),
        (c[0]-r/2,c[1]+r/4),
        ])

    # make shoulders for lanes
    # top left shoulder
    Wall([
        (0,0),
        (c[0]-r/4,0),
        (c[0]-r/4,c[1]-r/2-w),
        (c[0]-r/2-w,c[1]-r/4),
        [0,c[1]-r/4]
    ])

    # top right shoulder
    Wall([
        (screen_width,0),
        (screen_width-c[0]+r/4,0),
        (screen_width-c[0]+r/4,c[1]-r/2-w),
        (screen_width-c[0]+r/2+w,c[1]-r/4),
        [screen_width,c[1]-r/4]
    ])

    # bottom left shoulder
    Wall([
        (0,screen_height),
        (c[0]-r/4,screen_height),
        (c[0]-r/4,screen_height-c[1]+r/2+w),
        (c[0]-r/2-w,screen_height-c[1]+r/4),
        [0,screen_height-c[1]+r/4]
    ])

    # bottom right shoulder
    Wall([
        (screen_width,screen_height),
        (screen_width-c[0]+r/4,screen_height),
        (screen_width-c[0]+r/4,screen_height-c[1]+r/2+w),
        (screen_width-c[0]+r/2+w,screen_height-c[1]+r/4),
        [screen_width,screen_height-c[1]+r/4]
    ])

    
make_roundabout(screen_center,400,200)

############################
# Driver
############################

class Driver:
    # drivers have:
    #   a car to control
    #   a perception of their car
    #   a personality for driving
    def __init__(self, car, perception=None, personality=None):
        self.car = car
        self.perception = perception
        self.personality = personality

    def set_goal_position(self, goal_position):
        self.goal_position = goal_position

    def scan_for_cars(self):
        # look for other drivers
        # returns a list of other cars and their state:
        #   acceleration delta (coasting, braking, accelerating)
        #   distance delta (approaching, driving away)
        pass

    def pathfind_to_point(self, point_of_interest):
        global nav_mesh
        # find a valid path to the desired point
        current_point = self.car.position

        path = find_path(current_point, self.goal_position)
        nav_exteriors = list(nav_mesh.exterior.coords)
        

        if PATHFIND_DEBUG: # draw path
            pass


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
cur_mouse_pos = (0,0)
temp_draw = None

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
            save_wall_points(wall_points)
            wall_points = []
            for line in temp_lines:
                canvas.delete(line)
            temp_lines = []

def on_mouse_move(event):
    global cur_mouse_pos
    cur_mouse_pos = (event.x, event.y)

root.bind('<Motion>', on_mouse_move)
root.bind("<Button-1>", on_mouse_click)


############################
# basic simulation
############################

car1 = Car((18,14),(100,screen_center[1]),0)

#wall1 = Wall([(200,40),(200,120),(220,120),(220,40)])

cars = [car1]

ms = 10 # milliseconds for each step of the simulation

def main_loop():
    global start_time
    global temp_draw
    global nav_mesh
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
        #print(time.perf_counter()-start_time)
        start_time = time.perf_counter()

    # show drawing
    if len(wall_points) > 0:
        if temp_draw:
            canvas.delete(temp_draw)
        temp_draw = canvas.create_line(wall_points[-1],cur_mouse_pos)

    # see if mouse is in nav mesh
    print(nav_mesh.contains(Point(cur_mouse_pos)))
main_loop()


root.mainloop()