'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
import copy
import math
import random
import argparse
from PIL import Image
import numpy as np
from pprint import pprint

import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from math import sin, cos, radians, pi

from graphics import *

args=[]


g_CYCLE_TIME = .100

# Parameters you might need to use which will be set automatically
MAP_SIZE_X = None
MAP_SIZE_Y = None

# Default parameters will create a 4x4 grid to test with
scale = 2
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5/scale # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375/scale # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
waypoints = []
# Track servo angle in radians
servo_angle = None

# Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
subscriber_odometry = None
subscriber_state = None
update_sim = None

# CONSTANTS 
CYCLE_TIME = 0.05 # In seconds


def _load_img_to_intensity_matrix(img_filename):
  '''
  Helper function to read the world image containing obstacles
  YOu should not modify this
  '''
  global MAP_SIZE_X, MAP_SIZE_Y

  if img_filename is None:
      grid = np.zeros([800,1200])
      return grid

  img = Image.open(img_filename)

  MAP_SIZE_X = img.width
  MAP_SIZE_Y = img.height

  grid = np.zeros([img.height, img.width])
  for y in range(img.height):
      for x in range(img.width):
          pixel = img.getpixel((x,y))
          grid[y,x] = 255 - pixel[0] # Dark pixels have high values to indicate being occupied/having something interesting

  return grid


def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(x // g_MAP_RESOLUTION_X), int(y // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  '''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  '''
  # print('source:',vertex_source)
  # print('dest:',vertex_dest)
  if g_WORLD_MAP[vertex_source] == 1 or g_WORLD_MAP[vertex_dest] == 1:
    cost_val = 1000
  else:
    cost_val = 1

  # print('cost val: ', ost_val)

  return cost_val

def check_neighbors(vertex):
  first_row = list(range(g_NUM_X_CELLS))
  last_row = list(range(((g_NUM_X_CELLS * g_NUM_Y_CELLS)-g_NUM_X_CELLS), (g_NUM_X_CELLS * g_NUM_Y_CELLS)))
  right = vertex + 1
  left = vertex - 1
  down = vertex - g_NUM_X_CELLS
  up = vertex + g_NUM_X_CELLS
  neighbors = []

  if vertex == 0:  #first element
    first_index = up
    second_index = right  #check to the right and immediately below
    neighbors.append(first_index)
    neighbors.append(second_index)
  elif vertex == (g_NUM_X_CELLS * g_NUM_Y_CELLS -1): #last element
    first_index = left
    second_index = down
    neighbors.append(first_index)
    neighbors.append(second_index)
  elif vertex in first_row:
    first_index = up
    second_index = right
    third_index = left
    neighbors.append(first_index)
    neighbors.append(second_index)
    neighbors.append(third_index)
  elif vertex in last_row:
    if vertex%g_NUM_X_CELLS == 0: #first column
      second_index = right
      third_index = down
      neighbors.append(second_index)
      neighbors.append(third_index)
    elif (vertex+1)%(g_NUM_X_CELLS) == 0: #last column
      first_index = left
      third_index = down
      neighbors.append(first_index)
      neighbors.append(third_index)
    else:
      first_index = right
      second_index = left
      third_index = down
      neighbors.append(first_index)
      neighbors.append(second_index)
      neighbors.append(third_index)
  elif vertex%g_NUM_X_CELLS == 0: #first column
    first_index = up
    second_index = right
    third_index = down
    neighbors.append(first_index)
    neighbors.append(second_index)
    neighbors.append(third_index)
  elif (vertex+1)%g_NUM_X_CELLS == 0: #last column
    first_index = up
    second_index = left
    third_index = down
    neighbors.append(first_index)
    neighbors.append(second_index)
    neighbors.append(third_index)
  else:  #not first or last element or in first row or in last or in first column or last
    first_index = up
    second_index = right
    third_index = left
    fourth_index = down
    neighbors.append(first_index)
    neighbors.append(second_index)
    neighbors.append(third_index)
    neighbors.append(fourth_index)
  
  return neighbors


def run_dijkstra(source_vertex):
  '''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run
  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
  '''
  global g_NUM_X_CELLS, g_NUM_Y_CELLS

  # Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
  dist = [1000] * g_NUM_X_CELLS * g_NUM_Y_CELLS

  # Queue for identifying which vertices are up to still be explored:
  # Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
  #Q_cost = [1000] * g_NUM_X_CELLS*g_NUM_Y_CELLS
  Q_cost = []

  # Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
  prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS
  min_vals = []

  # Insert your Dijkstra's code here. Don't forget to initialize Q_cost properly!

  for i in range(g_NUM_X_CELLS):
    for j in range(g_NUM_Y_CELLS):
      v = ij_to_vertex_index(j,i)
      if v != source_vertex:
        Q_cost.append((v, 1000))
      else:
        Q_cost.append((v,0))
  # print(source_vertex)
  dist[source_vertex] = 0
  total_cost = 0

  while len(Q_cost) != 0:
    u = min(Q_cost)
    Q_cost = [my_tuple for my_tuple in Q_cost if my_tuple != u]
    u = u[0]
    neighbors = check_neighbors(u)
    #print("neighbors: ", neighbors)
    for v in neighbors:
      c = get_travel_cost(u, v)
      if c == 1000:
        length = 2000
      else:
        length = 1
      new_v = dist[u] + length
      if new_v < dist[v]:
        dist[v] = new_v
        prev[v] = u
        Q_cost = [my_tuple for my_tuple in Q_cost if my_tuple[0] != v]
        Q_cost.append((v, new_v))
        # print("q cost: ", Q_cost)
    total_cost += 1

  # print("final prev: ", prev)
  # print("final_dist: ", dist)
  return(prev)


def reconstruct_path(prev, source_vertex, dest_vertex):
  '''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
  '''
  final_path = [dest_vertex]
  print("final path: ", final_path)
  print("source vertex: ", source_vertex)

  current_vertex = dest_vertex
  print("current_vertex before while: ", current_vertex)


  while current_vertex != source_vertex:
    prev_vertex = prev[current_vertex]
    if prev_vertex == current_vertex:
      print("error: prev = current, will loop")
      break
    else:
      current_vertex = prev_vertex
    
    print("current vertex in while: ", current_vertex)

    if current_vertex != -1:
      final_path.append(current_vertex)
    else:
      print("error: hit -1 in prev. this should not happen")
      return []
      
    
  final_path.reverse()
  print("final path at end: ", final_path)
  return final_path


def render_map(map_array):
  '''
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example:
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0,
                       0, 0, 0, 0,
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  .
      .  .  .  .
      . [ ][ ] .
      .  . [ ] .


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  '''
  for j in range(g_NUM_Y_CELLS-1,-1, -1):
  	line_str = ''
  	for i in range(g_NUM_X_CELLS):
  		vertex = ij_to_vertex_index(i,j)
  		if g_WORLD_MAP[vertex] == 0:
  			line_str += ' .  '
  		else:
  			line_str += '[ ] '
  	print(line_str)

  for j in range(g_NUM_Y_CELLS-1,-1, -1):
  	line_str = ''
  	for i in range(g_NUM_X_CELLS):
  		vertex = ij_to_vertex_index(i,j)
  		if vertex<10:
  			vertex = str(vertex) + ' '
  		line_str += ' ' + str(vertex) + ' '
  	print(line_str)


def part_2(args):
  global g_dest_coordinates
  global g_src_coordinates
  global g_WORLD_MAP

  g_src_coordinates = (args.src_coordinates[0], args.src_coordinates[1])
  g_dest_coordinates = (args.dest_coordinates[0], args.dest_coordinates[1])

  # pixel_grid has intensity values for all the pixels
  # You will have to convert it to the earlier 0 and 1 matrix yourself
  pixel_grid = _load_img_to_intensity_matrix(args.obstacles)

  '''
  1) Compute the g_WORLD_MAP -- depending on the resolution, you need to decide if your cell is an obstacle cell or a free cell.
  2) Run Dijkstra's to get the plan
  3) Show your plan/path on the image
  Feel free to add more helper functions
  '''


  #image size
  num_pixels_x = len(pixel_grid[0])
  num_pixels_y = len(pixel_grid)
  #make g_WORLD_MAP
  for i in range(g_NUM_X_CELLS):
  	for j in range(g_NUM_Y_CELLS):
  		# coordinates of box to be searched for obstacles
  		start_coord_x = i * int(num_pixels_x//g_NUM_X_CELLS)
  		end_coord_x = (i+1) * int(num_pixels_x//g_NUM_X_CELLS)
  		end_coord_y = num_pixels_y - j * int(num_pixels_y//g_NUM_Y_CELLS)
  		start_coord_y = num_pixels_y - (j+1) * int(num_pixels_y//g_NUM_Y_CELLS)

  		#obstacle search 
  		filled = False
  		for x in range(start_coord_x, end_coord_x):
  			for y in range(start_coord_y, end_coord_y):
  				if pixel_grid[y][x]  != 0: # any non-white pixel means there is an obstacle in that cell
  					filled = True
  					pixel_grid[y][x] = 0 #recolor to black for export
  				else:
  					pixel_grid[y][x] = 255 #recolor to white for export
  		# update world map 
  		vertex = ij_to_vertex_index(i,j)
  		g_WORLD_MAP[vertex] = int(filled)


  start_x = float(g_src_coordinates[0])
  start_y = float(g_src_coordinates[1])

  start_i, start_j = xy_coordinates_to_ij_coordinates(start_x, start_y)
  start_vertex = ij_to_vertex_index(start_i, start_j)

  end_x = float(g_dest_coordinates[0])
  end_y = float(g_dest_coordinates[1])
  end_i, end_j = xy_coordinates_to_ij_coordinates(end_x, end_y)
  end_vertex= ij_to_vertex_index(end_i, end_j)

  render_map(g_WORLD_MAP)
  print( start_vertex)
  print( end_vertex)

  #RUN DIJKSTRA

  prev = run_dijkstra(start_vertex)
  final_path = reconstruct_path(prev, start_vertex, end_vertex)

  #RETURN WAYPOINTS
  final_x_y = []
  for vert in final_path:
  	i,j = vertex_index_to_ij(vert)
  	x,y = ij_coordinates_to_xy_coordinates(x,y)
  	final_x_y.append((x,y))
  print(final_x_y)
  return final_x_y



def calcDistanceError(dest_pose_x,dest_pose_y):
  dX = dest_pose_x - pose2d_sparki_odometry.x
  dY = dest_pose_y - pose2d_sparki_odometry.y
  d_err = math.sqrt((dX*dX) + (dY*dY))
  return d_err


def calcBearingError(dest_pose_x, dest_pose_y):
  #atan2f returns radians, dest_pose_theta is in radians
  b_err = math.atan2((dest_pose_y -pose2d_sparki_odometry.y),(dest_pose_x- pose2d_sparki_odometry.x)) - pose2d_sparki_odometry.theta
  return b_err

def bearing_error(dest_pose_x, dest_pose_y):
	while calcBearingError(dest_pose_x, dest_pose_y):
		msg = Float32MultiArray()
		msg.data = [-1.0, 1.0]
		publisher_motor.publish(msg)
		update_sim.publish(Empty())

def main():
    global publisher_motor, publisher_odom, update_sim
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry

    #Init your node to register it with the ROS core
    init()
    current_waypoint_index = 0
    while not rospy.is_shutdown():
    	time_start = time.time()
    	if current_waypoint_index < len(waypoints)-1:
    		dest_x = waypoints[current_waypoint_index][0]
    		dest_y = waypoints[current_waypoint_index][1]

    		bearing_error(dest_x, dest_y)
    		distance_error(dest_x, dest_y)

    		current_waypoint_index += 1


        time_end = time.time()
        rospy.sleep(CYCLE_TIME- (time_end-time_start))



def init(args):
    global publisher_motor, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global update_sim
    global waypoints
    rospy.init_node('sparki', anonymous = True)

    #Set up your publishers and subscribers
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size = 10);
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size = 10)
    update_sim = rospy.Publisher('/sparki/render_sim', Empty, queue_size = 10)

    rospy.sleep(1)

    # Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    # Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    
    waypoints = part2(args)
    rospy.sleep(1)
    update_sim.publish(Empty())    

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry
    pose2d_sparki_odometry = Pose2D()
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta


def convert_robot_coords_to_world(x_r, y_r):
    # Using odometry, convert robot-centric coordinates into world coordinates
    if x_r != None and y_r != None:
    	x_w = -x_r * cos((pi/2)-pose2d_sparki_odometry.theta) + y_r * cos(pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.x
    	y_w =  x_r * cos(pose2d_sparki_odometry.theta) + y_r * cos((pi/2)-pose2d_sparki_odometry.theta) + pose2d_sparki_odometry.y
    else:
    	x_w = None
    	y_w = None
    return x_w, y_w



if __name__ == "__main__":
  global args
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-s','--src_coordinates', nargs=2, default=[1.2, 0.2], help='Starting x, y location in world coords')
  parser.add_argument('-g','--dest_coordinates', nargs=2, default=[0.3, 0.7], help='Goal x, y location in world coords')
  parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()
  main()



