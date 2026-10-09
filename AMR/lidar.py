import pybullet
import math
import numpy as np

class LidarSensor:
    def __init__(self, robot_id, sensor_link_idx=0, num_rays=240, ray_length=5.0, ray_min=0.0):
        """
        TurtleBot LiDAR settings:
        sensor_link_idx=0 (base_link) - TurtleBot has no dedicated LiDAR link
        """
        self.robot_id = robot_id
        self.sensor_link_idx = sensor_link_idx
        self.num_rays = num_rays
        self.ray_length = ray_length
        self.ray_min = ray_min
        
        # Define ray angles (0 to 2pi)
        self.ray_angles = np.linspace(0, 2*math.pi, num_rays, endpoint=False)
        self.ray_from = []
        self.ray_to = []
        
        # Store debug line IDs for efficient updates
        self.debug_line_ids = None
        # TurtleBot body radius is ~0.176m, use slightly larger offset
        self.ray_offset = 0.20

    def scan(self,debug_draw=False):

        # Get base link pose (TurtleBot uses base_link)
        if self.sensor_link_idx == 0:
            
            base_pos, base_orn = pybullet.getBasePositionAndOrientation(self.robot_id)
            sensor_pos = base_pos
            sensor_orn = base_orn
        else:
            link_state = pybullet.getLinkState(self.robot_id, self.sensor_link_idx)
            sensor_pos = link_state[0]
            sensor_orn = link_state[1]
        

        ray_z=0.15

        rm=pybullet.getMatrixFromQuaternion(sensor_orn)
        rot_matrix=np.array([[rm[0],rm[1],rm[2]],
                             [rm[3],rm[4],rm[5]],
                             [rm[6],rm[7],rm[8]]

        ])

        self.ray_from=[]
        self.ray_to=[]

        for angle in self.ray_angles:
            # Ray in local sensor frame
            ray_dir_local = np.array([math.cos(angle), math.sin(angle), 0])
            # Rotate ray to world frame
            ray_dir_world = rot_matrix.dot(ray_dir_local)
            
            # Start rays from sensor center with offset to clear body
            start = [sensor_pos[0] + ray_dir_world[0] * self.ray_offset, 
                     sensor_pos[1] + ray_dir_world[1] * self.ray_offset, 
                     ray_z]
            end = [sensor_pos[0] + ray_dir_world[0] * self.ray_length,
                   sensor_pos[1] + ray_dir_world[1] * self.ray_length,
                   ray_z]
            
            self.ray_from.append(start)
            self.ray_to.append(end)

        results=pybullet.rayTestBatch(self.ray_from,self.ray_to)

        distances=[]
        hit_positions=[]

        for i, result in enumerate(results):
            hit_fraction = result[2]
            hit_pos = result[3]
            
            # Use offset in distance calculation
            dist = self.ray_offset + hit_fraction * (self.ray_length - self.ray_offset)
            distances.append(dist)
            
            if hit_fraction < 1.0:
                hit_positions.append(hit_pos)
            
            # Draw the rays
            if debug_draw:
                is_hit = hit_fraction < 1.0
                color = [1, 0, 1] if is_hit else [0, 1, 1] 
                end_point = [hit_pos[0], hit_pos[1], ray_z] if is_hit else [self.ray_to[i][0], self.ray_to[i][1], ray_z]
                
                if self.debug_line_ids is None:
                    self.debug_line_ids = {}
                
                if i not in self.debug_line_ids:
                    line_id = pybullet.addUserDebugLine(self.ray_from[i], end_point, color, lineWidth=1)
                    self.debug_line_ids[i] = line_id
                else:
                    pybullet.addUserDebugLine(self.ray_from[i], end_point, color, lineWidth=1,
                                              replaceItemUniqueId=self.debug_line_ids[i])
                
        return np.array(distances), np.array(hit_positions), self.ray_from, self.ray_to, results
    
