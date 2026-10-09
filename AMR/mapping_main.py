import pybullet
import time
import cv2
import numpy as np
import math
from environment import AMREnvironment
from lidar import LidarSensor
from odometry import WheelOdometry
from mapping import OccupancyGridMap

def main():
    #MODE="MAPPING"
    env=AMREnvironment(gui=True)
    obstacles=env.create_maze()
    env.setup_camera()

    LEFT_WHEEL_IDX=0
    RIGHT_WHEEL_IDX=1

    car_start_pos=[0.5,0.5,0.0]
    car_start_orn=pybullet.getQuaternionFromEuler([0,0,0])
    car_id=pybullet.loadURDF("../URDFS/turtlebot.urdf",car_start_pos,car_start_orn)

    print("\n=== TurtleBot Joint Info ===")
    for j in range(pybullet.getNumJoints(car_id)):
        info = pybullet.getJointInfo(car_id, j)
        print(f"Joint {j}: {info[1].decode('utf-8')} (type={info[2]})")
    print("============================\n")

    lidar=LidarSensor(car_id,sensor_link_idx=0)
    odom=WheelOdometry(initial_pose=[car_start_pos[0],car_start_pos[1],0],
                       wheel_radius=0.035,wheel_tread=0.23)
    map_grid=OccupancyGridMap(width_m=5.0,height_m=5.0,resolution=0.05)

    v=0.0
    w=0.0

    SPEED=3.0

    TURN_SPEED=2.5

    print("Started to Map!!")

    print("UP=forward, DOWN=backward, LEFT=turn left, RGHT=turn right. Q=exit and save")

    try:
         while pybullet.isConnected():
        
            keys = pybullet.getKeyboardEvents()
            
            if not (pybullet.B3G_UP_ARROW in keys and keys[pybullet.B3G_UP_ARROW] & pybullet.KEY_IS_DOWN) and \
               not (pybullet.B3G_DOWN_ARROW in keys and keys[pybullet.B3G_DOWN_ARROW] & pybullet.KEY_IS_DOWN):
                v = v * 0.9
            
            if not (pybullet.B3G_LEFT_ARROW in keys and keys[pybullet.B3G_LEFT_ARROW] & pybullet.KEY_IS_DOWN) and \
               not (pybullet.B3G_RIGHT_ARROW in keys and keys[pybullet.B3G_RIGHT_ARROW] & pybullet.KEY_IS_DOWN):
                w = w * 0.9 

            if ord('q') in keys and keys[ord('q')] & pybullet.KEY_WAS_TRIGGERED:
                break
            
            ACCEL = 0.3 
            
            if pybullet.B3G_UP_ARROW in keys and keys[pybullet.B3G_UP_ARROW] & pybullet.KEY_IS_DOWN:
                v = v + (SPEED - v) * ACCEL
            elif pybullet.B3G_DOWN_ARROW in keys and keys[pybullet.B3G_DOWN_ARROW] & pybullet.KEY_IS_DOWN:
                v = v + (-SPEED - v) * ACCEL
                
            if pybullet.B3G_LEFT_ARROW in keys and keys[pybullet.B3G_LEFT_ARROW] & pybullet.KEY_IS_DOWN:
                w = w + (TURN_SPEED - w) * ACCEL
            elif pybullet.B3G_RIGHT_ARROW in keys and keys[pybullet.B3G_RIGHT_ARROW] & pybullet.KEY_IS_DOWN:
                w = w + (-TURN_SPEED - w) * ACCEL
            
            if abs(v) > 0.1 or abs(w) > 0.1:
                print(f"\rv={v:.2f}, w={w:.2f}", end="")
            
            #Move the turtlebot

            right_wheel_v=v+w*odom.wheel_tread/2
            left_wheel_v=v-w*odom.wheel_tread/2

            pybullet.setJointMotorControl2(car_id,LEFT_WHEEL_IDX,pybullet.VELOCITY_CONTROL,targetVelocity=left_wheel_v,force=10)
            pybullet.setJointMotorControl2(car_id,RIGHT_WHEEL_IDX,pybullet.VELOCITY_CONTROL,targetVelocity=right_wheel_v,force=10)

            for _ in range(10):
                pybullet.stepSimulation()

            left_angle=pybullet.getJointState(car_id,LEFT_WHEEL_IDX)[0]
            right_angle=pybullet.getJointState(car_id,RIGHT_WHEEL_IDX)[0]

            pose_odom=odom.update(left_angle,right_angle)

            distances,hits,r_from,r_to,r_results=lidar.scan(debug_draw=True)

            map_grid.update_map(pose_odom,r_from,r_to,r_results)

            vis_map=map_grid.get_visualization()
            vis_map_large=cv2.resize(vis_map,(500,500),interpolation=cv2.INTER_NEAREST)

            cv2.imshow("AMR MAPPING",vis_map_large)
            cv2.waitKey(1)

    except Exception as e:
        import traceback
        print(f"Error during Simulation: {e}")
        traceback.print_exc()
    
    finally:
        map_grid.save_map("AMR/amr_map.npy")
        print("MAP SAVED!!")
        cv2.destroyAllWindows()
        env.disconnect()
if __name__ == "__main__":
    main()
