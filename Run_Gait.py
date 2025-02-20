from Dynamixel_Control.multi_tendon_sync_rw import MultiTendonSyncRW
from Gait.Gait_Generation import *
import numpy as np
import time

def main():
    x_swing,y_swing,x_stance,y_stance = plot_foot_path(60,35,3)
    q1,t1 = calculate_radii_front(x_swing,y_swing,x_stance,y_stance)
    q2,t2 = calculate_radii_back(x_swing,y_swing,x_stance,y_stance)
    t3 = t1[15:] + t1[:15]
    t4 = t2[15:]+ t2[:15]
    print(len(t3),len(t1))
    plot_beams(q1,106,106)
    Robot = MultiTendonSyncRW()
    Robot.set_zero_offsets_to_current_position()
    Robot.async_set_tendons_mm([0,-t1[0][0],-t1[0][1],0,t3[0][0],t3[0][1],0,-t2[0][0],-t2[0][1],0,t4[0][0],t4[0][1]])
    time.sleep(10)
    for k in range(10):
        for i in range(len(t1)):
            Robot.async_set_tendons_mm([0,-t1[i][0],-t1[i][1],0,t3[i][0],t3[i][1],0,-t2[i][0],-t2[i][1],0,t4[i][0],t4[i][1]])
            time.sleep(0.1)
    time.sleep(2)
    Robot.async_set_tendons_mm([0,0,0,0,0,0,0,0,0,0,0,0])
    time.sleep(2)
if __name__ == "__main__":
    main()

    
    
