from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_VELOCITY = 0.1


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                    wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_VELOCITY,
                                                    max_lat_accel, max_steer_angle)

        # Experimental values from WAlkthrough video TODO
        kp = 0.3
        ki = 0.1
        kd = 0.
        min_throttle = 0. # Minimum Throttle
        max_throttle = 0.2 # Maximum Throttle

        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        tau = 0.5  # cutoff frequency = 1/(2pi*tau)
        sample_time = 0.02 # Sample time
        # Filter the noise from the Velocity
        self.vel_lpf = LowPassFilter(tau, sample_time)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        # make sure the current velocity isnt changing drastically
        current_vel = self.vel_lpf.filt(current_vel)

        # get steering angle
        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.

        if linear_vel == 0. and current_vel < MIN_VELOCITY:
            throttle = 0.
            brake = 700  # Nm - to hold the car in place, if stopped
        elif throttle < 0.1 and vel_error < 0.:
            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Nm Torque

        # TODO: check if the the way points are not followed. Waypoint follower needs to be updated.
        # Check if the current and proposed and current Yaw is different.. refer the video last 5 mins

        return throttle, brake, steer
