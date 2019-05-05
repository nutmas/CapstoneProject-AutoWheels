from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle, stop_brake_torque):

        self.yaw_controller = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)

        kp = 0.9
        ki = 0.007
        kd = 0.2
        mn = 0.0 # Minimum throttle.
        mx = 0.2 # Maximum throttle.
        self.throttle_controller = PID(kp, ki, kd, mn, mx) 

        tau_throttle = 0.5
        ts_throttle =  0.02 # Sample time.
        self.throttle_lpf = LowPassFilter(tau_throttle, ts_throttle)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.fuel_mass = self.fuel_capacity * GAS_DENSITY
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.stop_brake_torque = stop_brake_torque
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.throttle_lpf.reset()
            return 0, 0, 0
        
        current_vel = self.throttle_lpf.filt(current_vel)        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = self.stop_brake_torque # Nm
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * (self.vehicle_mass + self.fuel_mass) * self.wheel_radius # Torque N*m
        return throttle, brake, steering
