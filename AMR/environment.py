import pybullet
import pybullet_data
import math
import random
import numpy as np

class AMREnvironment:
    def __init__(self,gui=True,time_step=1./240.):
        if gui:
            self.physics_client=pybullet.connect(pybullet.GUI)
        else:
            self.physics_client=pybullet.connect(pybullet.DIRECT)

        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.time_step=time_step
        self.reset()

    def reset(self):
        pybullet.resetSimulation()
        pybullet.setGravity(0.0,0.0,-9.8)
        pybullet.setTimeStep(self.time_step)
        self.plane_id=pybullet.loadURDF("plane.urdf")

    def create_maze(self, wall_size=5, wall_thickness=0.1, wall_height=0.5, obstacle_num=10, box_size_min=0.1, box_size_max=0.3, seed=10):
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        # Create walls
        wall_collision_id = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[wall_size/2, wall_thickness/2, wall_height/2])
        wall_visual_id = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[wall_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
        
        pybullet.createMultiBody(0, wall_collision_id, wall_visual_id, [wall_size/2, -wall_thickness/2, wall_height/2])
        pybullet.createMultiBody(0, wall_collision_id, wall_visual_id, [wall_size/2, wall_size+wall_thickness/2, wall_height/2])
        pybullet.createMultiBody(0, wall_collision_id, wall_visual_id, [-wall_thickness/2, wall_size/2, wall_height/2], pybullet.getQuaternionFromEuler([0, 0, math.pi/2]))
        pybullet.createMultiBody(0, wall_collision_id, wall_visual_id, [wall_size+wall_thickness/2, wall_size/2, wall_height/2], pybullet.getQuaternionFromEuler([0, 0, math.pi/2]))

        # Create static obstacles
        obstacles = []
        for _ in range(obstacle_num):
            box_size = random.uniform(box_size_min, box_size_max)
            x = random.uniform(0.5, wall_size - 0.5)
            y = random.uniform(0.5, wall_size - 0.5)
            yaw = random.uniform(0, math.pi)
            
            # Avoid starting position (around 0.5, 0.5)
            if math.sqrt((x-0.5)**2 + (y-0.5)**2) < 0.7:
                continue

            # Create box using primitives for perfect control over height
            # Height is fixed at 0.5m so Lidar at Z=0.2 always hits
            box_height = 0.5
            col_id = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[box_size/2, box_size/2, box_height/2])
            vis_id = pybullet.createVisualShape(pybullet.GEOM_BOX, halfExtents=[box_size/2, box_size/2, box_height/2], rgbaColor=[1, 0, 0, 1])
            
            obstacle_id = pybullet.createMultiBody(0, col_id, vis_id, [x, y, box_height/2], pybullet.getQuaternionFromEuler([0, 0, yaw]))
            obstacles.append({'id': obstacle_id, 'pos': [x, y], 'size': box_size, 'yaw': yaw})
        
        self.wall_size = wall_size
        return obstacles
    

    def setup_camera(self,distance=None,yaw=180,pitch=-89.9,target=None):
        if target is None:
            target=[self.wall_size/2,self.wall_size/2,0]
        if distance is None:
            distance=self.wall_size*1.2
        pybullet.resetDebugVisualizerCamera(distance,yaw,pitch,target)  

    def disconnect(self):
        if pybullet.isConnected():
            pybullet.disconnect()