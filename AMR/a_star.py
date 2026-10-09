from astar.search import AStar
import math
import numpy as np
import cv2

class AStarPlanner:
    def __init__(self,occupancy_grip,robot_radius=0.2):
        self.grid_map=occupancy_grip
        self.robot_radius=robot_radius
    def plan(self,start_world,goal_world):
        start_node=self.grid_map.world_to_map(start_world[0],start_world[1])
        goal_node=self.grid_map.world_to_map(goal_world[0],goal_world[1])

        if not (0<=start_node[0]<self.grid_map.width_px and 0<=start_node[1]<self.grid_map.height_px):
            print(f"Start Node {start_node} is out of the bound")
            return None
        if not (0<=goal_node[0]<self.grid_map.width_px and 0<=goal_node[1]<self.grid_map.height_px):
            print(f"Start Node {goal_node} is out of the bound")
            return None

        binary_grid=np.where(self.grid_map.grid==255,0,1).astype(np.uint8)

        inflate_pixels=int(self.robot_radius/self.grid_map.resolution)

        if inflate_pixels>0:
            kernel_size=2*inflate_pixels+1
            kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
            binary_grid=cv2.dilate(binary_grid,kernel,iterations=1)

        start_yx=(int(start_node[1]),int(start_node[0]))
        goal_yx=(int(goal_node[1]),int(goal_node[0]))

        if binary_grid[start_yx[0],start_yx[1]]==1:
            print("Start is in inflated Zone!")
            cv2.circle(binary_grid,(start_yx[1],start_yx[0]),3,0,-1)

        if binary_grid[goal_yx[0],goal_yx[1]]==1:
            print("Goal is in inflated Zone!")
            cv2.circle(binary_grid,(goal_yx[1],goal_yx[0]),3,0,-1)

        try:
            path_yx=AStar(binary_grid).search(start_yx,goal_yx)

            if path_yx is None:
                print("NO PATH FOUND BY A*")
                return None
            path_world=[]
            for y,x in path_yx:
                wx,wy=self.grid_map.map_to_world(x,y)
                path_world.append([wx,wy])
            return path_world
        except Exception as e:
            print(f"Error during A star search:, {e}")
            return None


        