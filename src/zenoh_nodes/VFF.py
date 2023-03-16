from sensor_msgs.msg import LaserScan
import math



class VFF:
    def __init__(self, laser_scan_msg: LaserScan, objetive: tuple):
        self.range = [laser_scan_msg.range_min, laser_scan_msg.range_max]
        self.ranges = list(laser_scan_msg.ranges)
        self.objetive = objetive
    
    def get_rep_force(self): #virtual repulsive force calculated for the virtual force field (VFF) algorithm
        forces = [(math.radians(theta), 1/dist if dist != 0.0 else dist) for theta, dist in enumerate(self.ranges)] #list with the atractive forces (tuples: (theta, r))
        #a=self.ranges[0]
        #print("rango 0:", a)
        rx, ry = 0, 0
        for force in forces: # sum up all the cartesian coordinates (negative to invert the direction)
            rx -= force[1] * math.cos(force[0])
            ry -= force[1] * math.sin(force[0])

        rep_force = [math.atan2(ry, rx), math.sqrt(rx**2 + ry**2)]
        return tuple(rep_force)

    def get_atr_force(self):
        pass
        #return get_force_from_point(self.objetive)

    def get_tot_force(): #sum up both forces having into account weights
        pass