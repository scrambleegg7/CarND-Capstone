
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from lowpass import LowPassFilter
from yaw_controller import YawController
from pid import PID
import rospy

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)


        kp = 0.3
        ki = 0.1
        kd = 0.0
        MIN_NUM = 0.0
        MAX_NUM = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM)

        tau = 0.5
        ts = .02
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.break_deadband = break_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return .0, .0, .0 

        current_vel = self.vel_lpf.filt( current_vel )

        steering = self.yaw_controller.get_steering( linear_vel, angular_vel, current_vel )

        vel_error = linear_vel = current_vel
        last_vel = current_vel


        current_time = rospy.get_time()
        sample_time = current_time - self.last_time


        throttle = self.throttle_controller.step( vel_error, sample_time  )

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0.0
            brake = 400
        
        elif throttle < 0.1 and current_vel < 0:
            throttle = 0
            decel = max(self.decel_limit, vel_error)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering
