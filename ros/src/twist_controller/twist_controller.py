import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

# General ideals for improvement:
# 1. Make full use of the variables, e.g. use mass ,accel_limit, decel_limit etc.
# 2. Use PID at the very beginning. But can we also use MPC? Are we allowed to subscribe /final_waypoints? Can try it if have time.
# 3. Gain scheduling.

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        min_speed = 0.1
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0
        mx = 1
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass+fuel_capacity*GAS_DENSITY
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.last_vel = 0

    def control(self, current_vel, linear_vel, angular_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0 and current_time < 0.1:  # Stop
            throttle = 0
            brake = 700  # 700 on Carla  # The torque to chill Carla
        elif throttle < 0.1 and vel_error < 0:  # Brake
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        rospy.loginfo('linear: %s, current: %s, error: %s',
                      linear_vel, current_vel, vel_error)
        rospy.loginfo('angular: %s', angular_vel)
        rospy.loginfo('throttle: %s, brake: %s, steer: %s',
                      throttle, brake, steering)
                      
        return throttle, brake, steering
