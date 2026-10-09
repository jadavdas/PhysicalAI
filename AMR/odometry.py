import math

class WheelOdometry:

    def __init__(self,initial_pose=[0,0,0],wheel_radius=0.035,wheel_tread=0.23):
        self.x,self.y,self.theta=initial_pose
        self.wheel_radius=wheel_radius
        self.wheel_tread=wheel_tread
        self.ONE_REVOLUTION=math.pi*self.wheel_radius*2

        self.last_right_angle=0
        self.last_left_angle=0

        self.first_run=True

    def update(self,current_left_angle,current_right_angle):
        if self.first_run:
            self.last_left_angle=current_left_angle
            self.last_right_angle=current_right_angle

            self.first_run=False
            return self.x,self.y,self.theta
        
        d_left=current_left_angle-self.last_left_angle
        d_right=current_right_angle-self.last_right_angle

        rev_left=d_left/(2*math.pi)
        rev_right=d_right/(2*math.pi)

        dist_left=rev_left*self.ONE_REVOLUTION
        dist_right=rev_right*self.ONE_REVOLUTION

        dist_avg=(dist_left+dist_right)/2.0

        self.x+=dist_avg*math.cos(self.theta)
        self.y+=dist_avg*math.sin(self.theta)
        self.theta+=math.atan2(dist_right-dist_left,self.wheel_tread)

        if self.theta>math.pi:
            self.theta-=2*math.pi
        elif self.theta<-math.pi:
            self.theta+=2*math.pi
        
        self.last_left_angle=current_left_angle
        self.last_right_angle=current_right_angle

        return self.x,self.y,self.theta

    def get_pose(self): return self.x,self.y,self.theta