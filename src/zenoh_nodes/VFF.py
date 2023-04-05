from sensor_msgs.msg import LaserScan
import math



class VFF:
    def __init__(self, laser_scan_msg: LaserScan, target_pos: tuple, max_dist: float, kps: tuple):
        self.range_min = laser_scan_msg.range_min
        self.range_max = min(laser_scan_msg.range_max, max_dist)
        self.range = [self.range_min, self.range_max] #max min ranges
        self.ranges = list(laser_scan_msg.ranges) #list of values

        self.target_pos = target_pos

        self.kp_rep, self.kp_atr = kps
        self.rep_force = tuple()
        self.atr_force = tuple()
        self.tot_force = tuple()
    
    def get_rep_force(self): #virtual repulsive force calculated for the virtual force field (VFF) algorithm
        #list with the atractive forces (tuples: (theta, r)):
        forces = [
            (
                math.radians(theta),
                1/(dist**2) if self.range_min < dist < self.range_max else 0.0
            )
            for theta, dist in enumerate(self.ranges)
        ]
        rx, ry = 0, 0
        for force_theta, force_r in forces: # sum up all the cartesian coordinates (negative to invert the direction due to repulsion)
            rx -= force_r * math.cos(force_theta)
            ry -= force_r * math.sin(force_theta)

        self.rep_force = (math.atan2(ry, rx), self.kp_rep*math.sqrt(rx**2 + ry**2))
        return self.rep_force

    def get_atr_force(self):
        obj_x, obj_y, obj_z = self.target_pos
        scale = -math.radians(90) #obj_x rescaled from [-1, 1] to [90, -90]º=[pi/2, -pi/2]rad
        self.atr_force = (obj_x * scale, self.kp_atr/obj_z if obj_z > 0.0 else 0.0)
        #self.atr_force = (obj_x**2 * scale, self.kp_atr/obj_z if obj_z > 0.0 else 0.0) # try
        return self.atr_force

    def get_tot_force(self): #sum up both forces having into account the weights
        rx, ry = 0.0, 0.0
        for force in [self.rep_force, self.atr_force]: # sum up all the cartesian coordinates (negative to invert the direction)
            rx += force[1] * math.cos(force[0])
            ry += force[1] * math.sin(force[0])
        self.tot_force = (math.atan2(ry, rx), math.sqrt(rx**2 + ry**2))
        return self.tot_force
    
    def get_forces(self):
        return (self.get_rep_force(), self.get_atr_force(), self.get_tot_force())
