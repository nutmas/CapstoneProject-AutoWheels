import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass=kwargs['vehicle_mass']
        self.fuel_capacity=kwargs['fuel_capacity']
        self.brake_deadband=kwargs['brake_deadband']
        self.decel_limit=kwargs['decel_limit']
        self.accel_limit=kwargs['accel_limit']
        self.wheel_radius=kwargs['wheel_radius']
        self.wheel_base=kwargs['wheel_base']
        self.steer_ratio=kwargs['steer_ratio']
        self.max_lat_accel=kwargs['max_lat_accel']
        self.max_steer_angle=kwargs['max_steer_angle']

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)


        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # mininum throttle value
        mx = 0.2  # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5  # 1/(2.pi.tau) = cutoff frequency
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)  # filter high frequency noise in velocity readings


        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
            
        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0.
            brake = 400  # N*m - to hold the car in place if we are stopped; acceleration ~ 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m

        return throttle, brake, steering
