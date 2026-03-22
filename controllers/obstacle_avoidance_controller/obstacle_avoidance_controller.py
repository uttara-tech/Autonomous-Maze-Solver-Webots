"""
    Author:      Uttara Naidu
    Date:        21/03/2026
    Description: iRobot Create Controller - A* Maze Navigator with distance sensors

    This controller implements autonomous navigation in a rectangular obstacle field using 
    distance sensors and the A* pathfinding algorithm.

    Functionalities:
    - Translates A* grid coordinates to real-world world-frame coordinates.
    - Calculates target headings and shortest-path angle error.
    - Commands discrete 90-degree turns to align with the estimated path.
    - Real-time logging of heading (Current vs. Target) and turn decisions.
"""

from controller import Robot
import numpy as np
import math

from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.grid import Grid
from pathfinding.core.diagonal_movement import DiagonalMovement


"""
     ----- Initializing global parameters ------

"""

MAXSPEED = 6.28
GRID_SIZE = 10                                      #number of tiles
WORLD_SIZE = 5.0                                    #rectangular arena of 5mX5m floor size 
grid = np.full((GRID_SIZE, GRID_SIZE), '?')         #creating an empty array with charater '?' to indicaate that the entire world is unexplored
OFFSET = WORLD_SIZE / 2                             #offset for coordinate shift 
CELL_SIZE = WORLD_SIZE / GRID_SIZE 
current_path = []
goal_grid = [0,0]                                   #target grid for A*
last_print_time = -1
full_path = []

"""
     ----- Initializing Devices ------

"""

robot = Robot()                                     #created the Robot instance.
timestep = int(robot.getBasicTimeStep())            #time step of the current world

gps = robot.getDevice('gps');                       #GPS device for localization
gps.enable(timestep)

compass = robot.getDevice('compass');               #Compass device for orientation
compass.enable(timestep)

left_motor = robot.getDevice('left wheel motor')    #for steering left wheel
left_motor.setPosition(float('inf'))
right_motor = robot.getDevice('right wheel motor')  #for steering right wheel
right_motor.setPosition(float('inf'))

sensors = ['ds_front',                              #Distance sensors for detecting obstacles
           'ds_left',
           'ds_right'
           ]
ds = []

for s in sensors:
    sensorDevice = robot.getDevice(s)
    sensorDevice.enable(timestep)
    ds.append(sensorDevice)


inertial_unit = robot.getDevice('inertial unit')    #for tracking rapid changes in movement and orientation 
inertial_unit.enable(timestep)
sensor_data = [
    (ds[0], 0.0),                                   #front sensor is at 0 radians relative to heading
    (ds[1], -0.7854),                               #left sensor is at -45 degrees
    (ds[2], 0.7854)                                 #right sensor is at +45 degrees
]


"""
     ----- Utility Functions ------

"""

def get_grid(x, y):
    """
        Function call to get current coordinates / grid number correspondind to a 10X10 matrix.
        E.g. if gx = 4, gy = 9 - it corresponds to -
            4th tile from top of the rectangular arena, and 
            9th column from left of the rectangular arena
    """
    gx = int((x + OFFSET) / CELL_SIZE)
    gy = int((y + OFFSET) / CELL_SIZE)
    
    return np.clip(gx, 0, 9), np.clip(gy, 0, 9)     #shifting the range from (-5.0 to 5.0) to (0.0 to 10.0)


def explored_cells(grid):
    return int(np.sum((grid == '0') | (grid == '1')))

def travelled_cells(grid):
    return int(np.sum((grid == '0')))

def turn90degrees(angle_error,left_obstacle,right_obstacle):
    """
        Function call to rotate the robot by 90 degrees (1.5708 radians).
        The direction of rotation depends on the angle error, calculated from A* path estimation
    """
    
    target_degrees = 90
    error_deg = math.degrees(angle_error)           #convert error to degrees for the log
    
    # Determine direction 

    if angle_error < -0.1:
        if(right_obstacle):
            # turn LEFT
            print('\n[INFO] FRONT and RIGHT obstacles detected.')
            print('\n[INFO] Turning LEFT...')
            left_speed = -0.5 * MAXSPEED
            right_speed = 0.5 * MAXSPEED
        else:
            # turn RIGHT
            print(f'\n[INFO] A* Path requires RIGHT turn (Error:{error_deg:.1f}°)')
            print('\n[INFO] Turning RIGHT...')
            left_speed = 0.5 * MAXSPEED
            right_speed = -0.5 * MAXSPEED         
    elif angle_error > 0:
        if (left_obstacle):
            print('\n[INFO] FRONT and LEFT obstacles detected.')
            print('\n[INFO] Turning RIGHT...')
            left_speed = 0.5 * MAXSPEED
            right_speed = -0.5 * MAXSPEED
        else:                                  
            # turn LEFT
            print(f'\n[INFO] A* Path requires LEFT turn (Error:{error_deg:.1f}°)')
            print('\n[INFO] Turning LEFT...')
            left_speed = -0.5 * MAXSPEED
            right_speed = 0.5 * MAXSPEED

    initial_yaw = inertial_unit.getRollPitchYaw()[2]    #for measuring rotation around z-axis             
    rotated_so_far = 0
    target_turn = math.radians(target_degrees)          #converting 90 degrees to radians

    while robot.step(timestep) != -1:                   #loop for rotating iRobot along its z-axis
        current_yaw = inertial_unit.getRollPitchYaw()[2]
        diff = current_yaw - initial_yaw
        if diff > math.pi: 
            diff -= 2* math.pi
        if diff < -math.pi: 
            diff += 2*math.pi
        rotated_so_far = abs(diff)
        
        if rotated_so_far >= target_turn:               #check to stop motors once the target is reached
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break
        else:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

    


def astar_path(grid,start_grid,end_grid):
    """
        Function call to determine the optimum path to the target grid (0,0) using A* algorithm.
    """

    walkable_map = np.ones((10, 10))                    #creating a walkable map 
    walkable_map[grid == '1'] = 0                       #in A*, 0:obstacle & 1:walkable
    
    grid = Grid(matrix=walkable_map)                    #initializing Grid for A*
    start = grid.node(start_grid[0], start_grid[1])     #changing as the robot moves through the rectangular arena
    end = grid.node(end_grid[0], end_grid[1])           #remains constant as target coordinates
    
    finder = AStarFinder()                              #initializing A*    (Reference: https://python-tcod.readthedocs.io/en/latest/tcod/path.html)
    path, _ = finder.find_path(start, end, grid)
    
    return path


def print_mapped_path(grid, path):
    """
        Function call to print A* path.
    """
    display_grid = np.copy(grid)
    
    
    if path and len(path) > 0:                          #planned A* path is shown with dots (.)
        for node in path:
            px, py = node.x, node.y
            if display_grid[py, px] == '?':             #print a dot only for cells with '?'
                display_grid[py, px] = '.'
    
    print('\n--- Map Matrix with A* Path (Dots) ---')   #print resulting map
    for row in display_grid:
        print(' '.join(row))



"""
     ----- Main loop ------

"""

print('\n[PROJECT] Real-time obstacle avoidance in Webots.\n')

while robot.step(timestep) != -1:
    pos = gps.getValues()                               #returns [x,y,z] coordinates of positioning
    comp_val = compass.getValues()                      #determine heading and the direction in which the compass is pointing

    if np.isnan(pos[0]): continue
    gx, gy = get_grid(pos[0], pos[1])                   #getting current coordinates of iRobot

    grid[gx, gy] = '0'                                  # '0' for Path travelled
    
    left_speed = 0.5 * MAXSPEED
    right_speed = 0.5 * MAXSPEED
    
    distanceSensors = [s.getValue() for s in ds]
    front_obstacle = distanceSensors[0] < 100           #setting threshold for front distance sensor
    left_obstacle = distanceSensors[1] < 100            #setting threshold for left distance sensor
    right_obstacle = distanceSensors[2] < 100           #setting threshold for right distance sensor
    
    theta = math.atan2(comp_val[1], comp_val[0])        #current orientation of iRobot 

    for sensor, offset_angle in sensor_data:
        if sensor.getValue() < 600:                     #setting threshold for detecting a wall / obstacle for mapping grid
            wall_x = pos[0] + 0.5 * math.sin(theta +    #calculating exactly where the wall is in the arena
                                             offset_angle)
            wall_y = pos[1] + 0.5 * math.cos(theta + 
                                             offset_angle)
            wx, wy = get_grid(wall_x, wall_y)           #getting grid indices
            
            if grid[wx, wy] != '0':                     #marking as obstacle '1' only if it's not already marked as '0'
                grid[wx, wy] = '1'


    if not current_path or (gx, gy) == current_path[0]:
        full_path = astar_path(grid, (gx, gy), goal_grid)
    if (gx, gy) == current_path[0] if current_path else False:
        current_path.pop(0) 
        
    if not current_path:
        full_path = astar_path(grid, (gx, gy), goal_grid)
        current_path = list(full_path) 

    if front_obstacle:                                                          #handling front obstacles
         
        print('\n|----------------------------------------------------|')       
        print(f'\n[BLOCK] Front obstacle detected.')
        full_path = astar_path(grid, (gx, gy), goal_grid)
        if len(current_path) > 0:
            target_gx, target_gy = current_path[0]
            gx,gy = get_grid(target_gx,target_gy)
            turn_angle = math.atan2(gy-pos[1],gx-pos[0])                        #calculating angle of rotation for iRobot
            angle_error = turn_angle - theta                                    #ideally detemines the error in angle of rotation for a robot - but in this code only used to determine direction of rotation for iRobot - 
                                                                                #-ve value corresponds to RIGHT turn
                                                                                #+ve value corresponds to LEFT turn
            angle_error = math.atan2(math.sin(angle_error), 
                                     math.cos(angle_error))                     #normalizing to range (-pi to +pi) and tracking path segments generated by A*
            
            #coverting angles in radians to degrees for the logs
            print(f'\n[INFO] Heading: {math.degrees(theta):.1f}° | Goal: {math.degrees(turn_angle):.1f}° | Error: {math.degrees(angle_error):.1f}°')

            left_speed = 0
            right_speed = 0

            turn90degrees(angle_error,left_obstacle,right_obstacle)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    if gx == 0 and gy == 0:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        final_time = robot.getTime()                                            #grabs the exact simulation timeReached! ---')
        print('\n--- SUCCESS: Goal (0,0) Reached! ---')
        print(f'\n--- BASELINE COMPETITION TIME: {final_time:2f} seconds ---') 
        total = GRID_SIZE*GRID_SIZE
        mapped = explored_cells(grid)
        travelled = travelled_cells(grid)
        print(f'\n--- CELLS MAPPED: {mapped}/{total} ({mapped * 100 // total}%) ---')
        print(f'\n--- CELLS TRAVELLED: {travelled}/{total} ---')
        print_mapped_path(grid, [])                                             #final Matrix Print
        break                                                                   #stop the while loop

    current_time = int(robot.getTime())
    if int(robot.getTime()) % 240 == 0 and current_time > last_print_time:       #periodically print mapping matrix
        total = GRID_SIZE*GRID_SIZE
        mapped = explored_cells(grid)
        travelled = travelled_cells(grid)
        print(f'\n--- CELLS MAPPED: {mapped}/{total} ({mapped * 100 // total}%) ---')
        print(f'\n--- CELLS TRAVELLED: {travelled}/{total} ---')
        print('\n--- Map Matrix ---')
        for row in grid:
            print(" ".join(row))
        print_mapped_path(grid,full_path)
        last_print_time = current_time
    

