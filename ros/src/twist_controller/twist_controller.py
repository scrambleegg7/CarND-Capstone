

import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
            accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
            
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        #mx = 0.2 # Maximum throttle value
        mx = 0.5 * accel_limit # Maximum throttle value
        
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        kp_cte = 0.5
        ki_cte = 0.0
        kd_cte = 0.2
        self.cte_controller = PID(kp_cte, ki_cte, kd_cte, -max_steer_angle, max_steer_angle)

        tau = 0.5 # 1/(2*pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, cte):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        #rospy.logwarn("twist_controller -->")
        #rospy.logwarn("current_vel: {0}".format(current_vel))
        #rospy.logwarn("dbw_enabled: {0}".format(dbw_enabled))
        #rospy.logwarn("linear_vel: {0}".format(linear_vel))
        #rospy.logwarn("angular_vel: {0}".format(angular_vel))


        if not dbw_enabled:
            # Reset controller so that the integral term does not accumulate.
            self.throttle_controller.reset()
            self.cte_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)
 
        vel_error = linear_vel - current_vel

        # Calculate time elapsed from the previous time step.
        current_time = rospy.get_time()        
        sample_time  = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        # Calculate the additional steering control due to CTE Error and add it to the base.
        steering_base = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering_cte  = self.cte_controller.step(cte, sample_time)
        steering_total= steering_base + steering_cte
        steering = max(min(self.max_steer_angle, steering_total), -self.max_steer_angle)
        #rospy.logwarn("steering: {0}".format(steering))
        
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 450  # Nm - for car to be held stationary

        elif throttle < 0.1 and vel_error < 0:
            throttle = 0          
            # Ensure deceleration does not exceed decel_limit.
            decel = max(vel_error, self.decel_limit)
            brake = (abs(decel)
                     * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                     * self.wheel_radius)  # Torque

        return throttle, brake, steering