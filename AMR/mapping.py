import numpy as np
import cv2
import math

class OccupancyGridMap:
    def __init__(self, width_m=5.0, height_m=5.0, resolution=0.05):
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution  # meters per pixel
        
        self.width_px = int(width_m / resolution)
        self.height_px = int(height_m / resolution)
        
        self.grid = np.full((self.height_px, self.width_px), 127, dtype=np.uint8)
        
        self.obstacle_radius_px = 2

    def world_to_map(self,x,y):
        mx=int(x/self.resolution)
        my=int(y/self.resolution)
        return mx,my
    def map_to_world(self,mx,my):
        x=mx*self.resolution
        y=my*self.resolution
        return x,y
    def update_map(self,robot_pos,ray_froms,ray_tos,results):
        
        for i, result in enumerate(results):
            start = ray_froms[i]
            hit_fraction = result[2]
            hit_pos = result[3]
            end = ray_tos[i]
            
            sx, sy = self.world_to_map(start[0], start[1])
            
            if hit_fraction < 1.0:
            
                hx, hy = self.world_to_map(hit_pos[0], hit_pos[1])
                cv2.line(self.grid, (sx, sy), (hx, hy), 255, 1)
                
                if 0 <= hx < self.width_px and 0 <= hy < self.height_px:
                    cv2.circle(self.grid, (hx, hy), self.obstacle_radius_px, 0, -1)
            else:
                ex, ey = self.world_to_map(end[0], end[1])
                cv2.line(self.grid, (sx, sy), (ex, ey), 255, 1)

        rx, ry = self.world_to_map(robot_pos[0], robot_pos[1])
        if 0 <= rx < self.width_px and 0 <= ry < self.height_px:
            cv2.circle(self.grid, (rx, ry), 3, 255, -1)

    def save_map(self, filename="map.npy"):
        
        save_grid = self.grid.copy()
        
        obstacle_mask = (save_grid < 100).astype(np.uint8)
        
    
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        dilated_obstacles = cv2.dilate(obstacle_mask, kernel, iterations=1)
        
        save_grid[dilated_obstacles == 1] = np.minimum(save_grid[dilated_obstacles == 1], 50)
        
        np.save(filename, save_grid)
        cv2.imwrite(filename.replace(".npy", ".png"), save_grid)
        print(f"Map saved to '{filename}' with obstacle dilation")

    def load_map(self, filename="map.npy"):
        self.grid = np.load(filename)
        self.height_px, self.width_px = self.grid.shape
        self.width_m = self.width_px * self.resolution
        self.height_m = self.height_px * self.resolution

    def get_visualization(self):
        return cv2.flip(self.grid, 0)
        
    def is_occupied(self, x, y, threshold=127):
        if x < 0 or y < 0 or x >= self.width_m or y >= self.height_m:
            return True
        mx, my = self.world_to_map(x, y)
        if 0 <= mx < self.width_px and 0 <= my < self.height_px:
            return self.grid[my, mx] < threshold
        return True