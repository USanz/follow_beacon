from sensor_msgs.msg import LaserScan
import math



class VFF:
    def __init__(self, laser_scan_msg: LaserScan, objetive: tuple):
        self.rep_force = tuple()
        self.atr_force = tuple()
        self.tot_force = tuple()

        self.range = [laser_scan_msg.range_min, laser_scan_msg.range_max]
        self.range_min, self.range_max = self.range
        self.ranges = list(laser_scan_msg.ranges)

        self.objetive = objetive
    
    def get_rep_force(self): #virtual repulsive force calculated for the virtual force field (VFF) algorithm
        #list with the atractive forces (tuples: (theta, r)):
        forces = [(math.radians(theta), 1/dist if dist != 0.0 else dist) for theta, dist in enumerate(self.ranges)]
        rx, ry = 0, 0
        for force in forces: # sum up all the cartesian coordinates (negative to invert the direction)
            rx -= force[1] * math.cos(force[0])
            ry -= force[1] * math.sin(force[0])

        self.rep_force = (math.atan2(ry, rx), math.sqrt(rx**2 + ry**2))
        return self.rep_force

    def get_atr_force(self):
        obj_x, obj_y, obj_z = self.objetive
        self.atr_force = (-math.asin(obj_x / obj_z), 1/obj_z if obj_z > 0.0 else 0.0)
        return self.atr_force

    def get_tot_force(self, kp_r, kp_a): #sum up both forces having into account weights
        rx, ry = 0.0, 0.0
        rep_force = (self.rep_force[0], kp_r * self.rep_force[1])
        atr_force = (self.atr_force[0], kp_a * self.atr_force[1])
        for force in [rep_force, atr_force]: # sum up all the cartesian coordinates (negative to invert the direction)
            rx += force[1] * math.cos(force[0])
            ry += force[1] * math.sin(force[0])
        self.tot_force = (math.atan2(ry, rx), math.sqrt(rx**2 + ry**2))
        return self.tot_force