import pybullet
import time
import cv2
import numpy as np
import math
import random
from environment import AMREnvironment
from lidar import LidarSensor
from odometry import WheelOdometry
from mapping import OccupancyGridMap
from a_star import AStarPlanner

goal_pos=None
update_goal=False

def mouse_callback(event, x, y, flags, param):
    """Convert click to world coordinates - simple direct mapping"""
    global goal_pos, update_goal
    if event == cv2.EVENT_LBUTTONDOWN:
        map_grid = param
        # Direct mapping: x -> world X, (500-y) -> world Y (image Y is inverted)
        wx = x * map_grid.width_m / 500
        wy = (500 - y) * map_grid.height_m / 500
        
        print(f"[CLICK] Screen ({x},{y}) -> World ({wx:.2f},{wy:.2f})")
        
        if 0 <= wx <= map_grid.width_m and 0 <= wy <= map_grid.height_m:
            goal_pos = [wx, wy]
            update_goal = True

def main():
    global goal_pos, update_goal

    #Setup of Environment
    env = AMREnvironment(gui=True)
    obstacles = env.create_maze(seed=10) 
    env.setup_camera(yaw=0, pitch=-89.9)  # Changed to yaw=0 to match map orientation

    #TurtleBot
    LEFT_WHEEL_IDX = 0
    RIGHT_WHEEL_IDX = 1
    
    car_start_pos = [0.5, 0.5, 0.0]
    car_start_orn = pybullet.getQuaternionFromEuler([0, 0, 0])
    car_id = pybullet.loadURDF("../URDFS/turtlebot.urdf", car_start_pos, car_start_orn)
    
    print("\n=== TurtleBot Navigation ===")
    robot_pos, robot_orn = pybullet.getBasePositionAndOrientation(car_id)
    print(f"Robot at ({robot_pos[0]:.2f},{robot_pos[1]:.2f})")


    #Initialize the modules
    lidar = LidarSensor(car_id, sensor_link_idx=0)
    odom = WheelOdometry(initial_pose=[robot_pos[0], robot_pos[1], 0], wheel_radius=0.035, wheel_tread=0.23)

    #Load Map
    map_grid=OccupancyGridMap(width_m=5.0,height_m=5.0,resolution=0.05)

    try:
        map_grid.load_map("AMR/amr_map.npy")
        print("Map Loaded")
    except:
        print("ERROR: Run he mapping_main.py first!!")
        env.disconnect()
        return
    
    planner=AStarPlanner(map_grid)
    global_path=None
    path_index=0
    goal_marker_id=None

    cv2.namedWindow("AMR Navigation")
    cv2.setMouseCallback("AMR Navigation", mouse_callback, map_grid)
    
    v, w = 0.0, 0.0
    
    print("Click on map to set goal. Press 'Q' to quit.\n")

    frame_count=0
    try:
        while pybullet.isConnected():
            frame_count+=1
            robot_pos, robot_orn = pybullet.getBasePositionAndOrientation(car_id)
            robot_euler = pybullet.getEulerFromQuaternion(robot_orn)
            rx, ry, rtheta = robot_pos[0], robot_pos[1], robot_euler[2]

            #Handle Goal
            if update_goal and goal_pos:
                if goal_marker_id:
                    pybullet.removeBody(goal_marker_id)
                vis_id = pybullet.createVisualShape(pybullet.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 1, 0, 1])
                goal_marker_id = pybullet.createMultiBody(baseVisualShapeIndex=vis_id, 
                                                           basePosition=[goal_pos[0], goal_pos[1], 0.2])
                
                print(f"[PATH] Planning ({rx:.2f},{ry:.2f}) -> ({goal_pos[0]:.2f},{goal_pos[1]:.2f})")
                new_path = planner.plan([rx, ry], goal_pos)
                if new_path:
                    global_path = new_path
                    path_index = 0  # Start from beginning of path
                    print(f"[PATH] Found {len(global_path)} waypoints")
                else:
                    print("[PATH] No path found!")
                    global_path = None
                update_goal = False

            #Navigation with lookahead and stuck detection
            target_pt = None
            if global_path and path_index < len(global_path):
                # LOOKAHEAD: Find a waypoint that's ahead but not too far
                lookahead_dist = 0.3  # Look 30cm ahead
                best_index = path_index
                for i in range(path_index, len(global_path)):
                    wp = global_path[i]
                    d = math.hypot(rx - wp[0], ry - wp[1])
                    if d > lookahead_dist:
                        best_index = i
                        break
                    elif d < 0.1:  # Skip waypoints we're very close to
                        best_index = i + 1
                
                path_index = min(best_index, len(global_path) - 1)
                target_pt = global_path[path_index]
                
                # Check if goal reached
                dist_to_goal = math.hypot(rx - goal_pos[0], ry - goal_pos[1])
                if dist_to_goal < 0.2:
                    print("[NAV] GOAL REACHED!")
                    global_path = None
                    v, w = 0.0, 0.0
                    if goal_marker_id:
                        pybullet.removeBody(goal_marker_id)
                        goal_marker_id = None
                    continue
                
                # Calculate steering
                dx = target_pt[0] - rx
                dy = target_pt[1] - ry
                target_angle = math.atan2(dy, dx)
                angle_err = target_angle - rtheta
                
                # Normalize to [-pi, pi]
                while angle_err > math.pi: angle_err -= 2*math.pi
                while angle_err < -math.pi: angle_err += 2*math.pi
                
                # P controller for steering
                w = 2.0 * angle_err
                w = max(-1.5, min(1.5, w))  # Reduced max turn rate
                
                # Speed control - SLOWER overall, even slower when turning
                if abs(angle_err) > 0.5:  # Big angle error
                    v = 0.05  # Very slow when turning
                elif abs(angle_err) > 0.2:
                    v = 0.07# Medium when aligning
                else:
                    v = 0.10 #Cruising speed
                
                # Debug
                if frame_count % 60 == 0:
                    print(f"[NAV] Pos:({rx:.2f},{ry:.2f}) -> WP[{path_index}]:({target_pt[0]:.2f},{target_pt[1]:.2f}) | v={v:.2f}, w={w:.2f}, goal_dist={dist_to_goal:.2f}m")
            else:
                v, w = 0.0, 0.0
            
            distances,hits,r_from,r_to,r_results=lidar.scan(debug_draw=True)

            num_rays=len(distances)
            front_range=30

            front_left_rays = list(range(0, front_range))  # 0 to 29
            front_right_rays = list(range(num_rays - front_range, num_rays))  # 210 to 239
            front_rays = front_left_rays + front_right_rays

            front_distances = [distances[i] for i in front_rays if i < len(distances)]
            if front_distances:
                min_front_dist = min(front_distances)
                
                # Safety thresholds
                STOP_DIST = 0.25   # Emergency stop distance
                SLOW_DIST = 0.40   # Slow down distance
                
                if min_front_dist < STOP_DIST:
                    # EMERGENCY: Too close, stop and turn
                    v = 0.0
                    # Find which side has more space and turn that way
                    left_avg = np.mean([distances[i] for i in front_left_rays if i < len(distances)])
                    right_avg = np.mean([distances[i] for i in front_right_rays if i < len(distances)])
                    w = 1.0 if left_avg > right_avg else -1.0
                    if frame_count % 30 == 0:
                        print(f"[AVOID] STOP! Obstacle at {min_front_dist:.2f}m, turning {'left' if w > 0 else 'right'}")
                        
                elif min_front_dist < SLOW_DIST:
                    # Slow down proportionally
                    slow_factor = (min_front_dist - STOP_DIST) / (SLOW_DIST - STOP_DIST)
                    v = v * slow_factor
                    if frame_count % 60 == 0:
                        print(f"[AVOID] Slowing: obstacle at {min_front_dist:.2f}m, v={v:.2f}")
            # ====== END AVOIDANCE ======
            # Apply wheel velocities (convert m/s to rad/s)
            wheel_radius = 0.035
            wheel_tread = 0.23
            left_w = (v - w * wheel_tread / 2) / wheel_radius
            right_w = (v + w * wheel_tread / 2) / wheel_radius
            
            pybullet.setJointMotorControl2(car_id, LEFT_WHEEL_IDX, pybullet.VELOCITY_CONTROL, 
                                           targetVelocity=left_w, force=20)
            pybullet.setJointMotorControl2(car_id, RIGHT_WHEEL_IDX, pybullet.VELOCITY_CONTROL, 
                                           targetVelocity=right_w, force=20)
            
            for _ in range(10):
                pybullet.stepSimulation()

            # Visualization - NO FLIPS, direct world-to-pixel mapping
            vis_map = map_grid.get_visualization()  # This has vertical flip built-in
            vis_map_large = cv2.resize(vis_map, (500, 500), interpolation=cv2.INTER_NEAREST)
            vis_map_color = cv2.cvtColor(vis_map_large, cv2.COLOR_GRAY2BGR)
            
            def to_px(x, y):
                """World coords to screen pixels"""
                px = int(x * 500 / map_grid.width_m)
                py = int((map_grid.height_m - y) * 500 / map_grid.height_m)
                return (px, py)
            cv2.circle(vis_map_color,to_px(rx,ry),8,(255,0,0),2)

            # Draw heading
            hx = rx + 0.3 * math.cos(rtheta)
            hy = ry + 0.3 * math.sin(rtheta)
            cv2.line(vis_map_color, to_px(rx, ry), to_px(hx, hy), (255, 0, 0), 2)
            
            # Draw path (Cyan)
            if global_path:
                for i in range(len(global_path) - 1):
                    cv2.line(vis_map_color, to_px(global_path[i][0], global_path[i][1]),
                             to_px(global_path[i+1][0], global_path[i+1][1]), (255, 255, 0), 2)
                
                # Current waypoint (Yellow star)
                if target_pt:
                    cv2.drawMarker(vis_map_color, to_px(target_pt[0], target_pt[1]), (0, 255, 255),
                                   markerType=cv2.MARKER_STAR, markerSize=12, thickness=2)
            
            # Draw goal (Green)
            if goal_pos:
                cv2.drawMarker(vis_map_color, to_px(goal_pos[0], goal_pos[1]), (0, 255, 0),
                               markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                
            # Status text
            cv2.putText(vis_map_color, f"v={v:.2f} w={w:.2f}", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.imshow("AMR Navigation", vis_map_color)
            if cv2.waitKey(20) == ord('q'):
                break
                
    except Exception as e:
        import traceback
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        env.disconnect()

if __name__ == "__main__":
    main()

            