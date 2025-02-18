from Dynamixel_Control.multi_tendon_sync_rw import MultiTendonSyncRW
import keyboard
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve
import numpy as np

class Body_Kinematics:
    def __init__(self,dim,posi,ori,footrec):
        self.w = dim[0]
        self.l = dim[1]
        self.x,self.y,self.z = posi[0],posi[1],posi[2]
        self.roll,self.pitch,self.yaw = ori[0],ori[1],ori[2]
        self.footrec_width, self.footrec_len  = footrec[0],footrec[1]
        self.corners_foot = np.array([
            [-self.footrec_width / 2, -self.footrec_len / 2, 0],
            [self.footrec_width / 2, -self.footrec_len / 2, 0],
            [self.footrec_width / 2, self.footrec_len / 2, 0],
            [-self.footrec_width / 2, self.footrec_len / 2, 0]
        ])

    def calculate_corners(self):
        x_c, y_c, z_c = self.x,self.y,self.z
        w, h = self.w,self.l
        roll, pitch, yaw = self.roll,self.pitch,self.yaw

        corners_local = np.array([
            [-w/2, -h/2, 0],
            [w/2, -h/2, 0],
            [w/2, h/2, 0],
            [-w/2, h/2, 0]
        ])
        
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        R = R_z @ R_y @ R_x

        corners_global = np.dot(corners_local, R.T) + np.array([x_c, y_c, z_c])
        
        return corners_global
    
    def calculate_vectors(self,corners_body):
        #v1 = [k1 - k2 for k1, k2 in zip(self.corners_foot[0], corners_body[0])]
        v1 = [self.corners_foot[0][0] - corners_body[0][0],self.corners_foot[0][1] - corners_body[0][1],self.corners_foot[0][2] - corners_body[0][2]]
        v2 = [k1 - k2 for k1, k2 in zip(self.corners_foot[1], corners_body[1])]
        v3 = [k1 - k2 for k1, k2 in zip(self.corners_foot[2], corners_body[2])]
        v4 = [k1 - k2 for k1, k2 in zip(self.corners_foot[3], corners_body[3])]
        return(v1,v2,v3,v4)

class Leg_Kinematics:
    def __init__(self,x,y,z,leg_length,plane_angle,mode):
        self.x = x #Target Foot Position
        self.y = y
        self.z = z
        self.l1 = leg_length[0]
        self.l2 = leg_length[1]
        self.theta = plane_angle    #Should be pi/9
        self.radius1 = -1           #Lateral
        self.radius2 = -1           #Proximal
        self.radius3 = -1           #Distal
        self.cross = 0              #In plane distance
        self.mode = mode            #"S" = S shape, "C" = C shape
        self.t = [0,0,0,0,0,0]

    def compute_lateral_bending(self):
        if self.x == 0:             
            self.cross = self.y
            self.radius1 = -1
            
        else:
            flat_plane = np.array([0,0,-1])
            axis = [1,0,0]  # Define the rotation axis (X axis)
            rotation = R.from_rotvec((np.pi - self.theta) * np.array(axis))  # Create rotation object
            rotated_vec = rotation.apply(flat_plane)
            point = np.array([self.x,self.y,self.z])
            dot_product = np.dot(rotated_vec, point)
            projection = point - (dot_product) / np.dot(rotated_vec,rotated_vec) * rotated_vec
            #print(projection)
            yp = projection[1]/(-np.cos(np.pi - self.theta))
            xp = projection[0]
            #print(xp,yp)
            r = (xp**2 + yp**2)/(2*xp)
            l = abs(r) * np.arctan(abs(yp)/(abs(r-xp)))
            self.radius1 = r
            self.cross = l 

    def compute_inplane_bending(self):
        if self.mode == "s":
            guess = [40,40]
            solution = fsolve(self.equations1, guess)

        if self.mode == "c":
            guess = [60,60]
            solution = fsolve(self.equations2, guess)

        r1, r2 = solution
        self.radius2 = r1
        self.radius3 = r2

    # Define the system of equations
    def equations1(self,vars):
        r1, r2 = vars
        eq1 = (-r1 * np.sin(self.theta) +
            (r1 + r2) * np.cos((self.l1 / r1) - (np.pi / 2) + self.theta) -
            r2 * np.cos((self.l2 / r2) - ((self.l1 / r1) - (np.pi / 2) + self.theta)) - self.cross)
        
        eq2 = (r1 * np.cos(self.theta) +
            (r1 + r2) * np.sin((self.l1 / r1) - (np.pi / 2) + self.theta) +
            r2 * np.sin((self.l2 / r2) - ((self.l1 / r1) - (np.pi / 2) + self.theta)) + self.z)
        return [eq1, eq2]
    
    def equations2(self, vars):
        r1, r2 = vars
        eq1 = ((-r1 * np.sin(self.theta)) - (r2-r1)*np.cos((np.pi/2)-self.theta-(self.l1/r1)) + 
            r2 * np.cos(self.l2/r2 - ((np.pi/2)-self.theta-(self.l1/r1))) - self.cross)
        eq2 = ((r1 * np.cos(self.theta)) + (r2-r1)*np.sin((np.pi/2)-self.theta-(self.l1/r1)) + 
            r2 * np.sin(self.l2/r2 - ((np.pi/2)-self.theta-(self.l1/r1))) + self.z)
        return [eq1, eq2]
    
    def compute_tendon_lengths(self):
        if self.radius1 == -1:
            t1,t2 = 0,0
        if self.radius1 < -1:
            t1 = -((self.radius1 - d)*(self.l1+self.l2)/self.radius1 - (self.l1+self.l2))
            t2 = -((self.radius1 + d)*(self.l1+self.l2)/self.radius1 - (self.l1+self.l2))        
        if self.radius1 >= 0:
            t1 = (self.radius1 - d)*(self.l1+self.l2)/self.radius1 - (self.l1+self.l2)
            t2 = (self.radius1 + d)*(self.l1+self.l2)/self.radius1 - (self.l1+self.l2)
        if self.radius2 < 0:
            t3,t4 = 0,0
        if self.radius2 >= 0:
            t3 = (self.radius2 - d)*(self.l1)/self.radius2 - self.l1
            t4 = (self.radius2 + d)*(self.l1)/self.radius2 - self.l1
        if self.radius3 < 0:
            t5,t6 = 0,0
        if self.radius3 >= 0:
            t5 = (self.radius3 - d)*(self.l2)/self.radius3 - self.l2
            t6 = (self.radius3 + d)*(self.l2)/self.radius3 - self.l2
        self.t = [t1,t2,t3,t4,t5,t6]
    
    def run_kinematics(self):
        self.compute_lateral_bending()
        self.compute_inplane_bending()
        self.compute_tendon_lengths()

    def output_results(self):
        print("Radii: ", self.radius1, self.radius2, self.radius3)
        print("Thetas: ", (self.l1+self.l2)/self.radius1 * 180 / np.pi, self.l1/self.radius2 * 180 / np.pi, self.l2/self.radius3 * 180 / np.pi)
        print("Tendons: ", self.t)
