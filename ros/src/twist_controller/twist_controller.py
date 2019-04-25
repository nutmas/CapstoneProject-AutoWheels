from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_control  = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)

        kp =  .3
        ki =  .2
        self.throttle_PID = PID(kp,ki, 0.,0.,1.) 

        tau = .5
        ts  = .2
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass   = vehicle_mass
        self.fuel_capacity  = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.wheel_radius   = wheel_radius
#       self.wheel_base     = wheel_base
#       self.steer_ratio    = steer_ratio
        self.last_time      = rospy.get_time()

    def control(self, current_velocity, dbw_enbaled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        throttle, brake, steer = 0., 0., 0.
        if dbw_enbaled:
            current_velocity = self.vel_lpf.filt(current_velocity)
            vel_error        = linear_vel - current_velocity
            self.last_vel    = current_velocity

            current_time   = rospy.get_time()
            sample_time    = current_time - self.last_time
            self.last_time = current_time
         
            throttle = self.throttle_PID.step(vel_error, sample_time)
            brake    = 0

            if linear_vel == 0. and vel_error < ONE_MPH:
                throttle = 0
                brake    = 400 # N*m - to hold the car in place a=-1m/s^2
            elif throttle < .1 and vel_error < 0:
                throttle = 0
                decel = max(vel_error, self.decel_limit)
                brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Tourque in N*m 
 
            steer = self.yaw_control.get_steering(linear_vel, angular_vel, current_velocity)        
        return throttle, brake, steer
