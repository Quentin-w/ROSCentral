import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

VEL_KP = 0.4
VEL_KI = 0.0
VEL_KD = 0.0

BR_KP = 0.3
BR_KI = 0.0
BR_KD = 0.0


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        self.vehicle_mass = vehicle_mass+fuel_capacity*GAS_DENSITY
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

	      # Using PID to control throttle and brake
        self.throttle_controller = PID(VEL_KP, VEL_KI, VEL_KD, mn=0, mx=1)
	      self.brake_controller = PID(BR_KP, BR_KI, BR_KD, mn=0, mx=1)
	      self.lp_filter = LowPassFilter(0.2, 1)

        # Control the steer: PID or YawController?
        # self.steer_controller = PID(
        #     ANG_KP, ANG_KI, ANG_KD, mn=-max_steer_angle, mx=max_steer_angle)
        min_speed = 0
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.last_time = rospy.get_time()
        # self.last_throttle = 0

    def control(self, desired_twist, real_twist, dbw_enabled):
        # def control(self, current_vel, linear_vel, angular_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Param:
        # desired_twist - desired speed (linear, angular)
        # real_twist - real speed (linear, angular)
        # dbw_enabled - whether manual control is engabled or not

        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
	          self.brake_controller.reset()
            return 0., 0., 0.

	      # Time
        current_time = rospy.get_time()
        sample_time = current_time-self.last_time
        self.last_time = current_time

        # Deal with throttle, brake, and steering
        vel_desired = desired_twist.twist.linear.x
        vel_real = real_twist.twist.linear.x
        ang_desired = desired_twist.twist.angular.z
        ang_real = real_twist.twist.angular.z

        vel_error = vel_desired - vel_real
        ang_error = ang_desired - ang_real

	      throttle = self.throttle_controller.step(vel_error, sample_time)
	      brake = self.brake_controller.step(-vel_error, sample_time)

        #if vel_desired == 0 and vel_real < 0.05:  # Stop
        #    self.throttle_controller.reset()
        #    throttle = 0
        #    steering = 0
        #    brake = self.brake_deadband  # 700 on Carla  # The torque to chill Carla
        #elif vel_error < 0:  # Brake
        #    throttle = 0
        #    decel = max(vel_error, self.decel_limit)
        #    brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        #else:  # Acceleration
        #    self.accel_limit
        #    throttle = self.throttle_controller.step(vel_error, sample_time)
        #    if throttle - self.last_throttle > MAX_THROTTLE_CHANGE:  # Limit the jerk
        #        throttle = self.last_throttle + MAX_THROTTLE_CHANGE
        #    self.last_throttle = throttle
        #    brake = 0

        steering = self.yaw_controller.get_steering(
            vel_desired, ang_desired, vel_real)
	      steering = self.lp_filter.filt(steering)

	      rospy.loginfo('linear: %s, current: %s, error: %s', vel_desired, vel_real, vel_error)
	      rospy.loginfo('angular: %s, current: %s, error: %s', ang_desired, ang_real, ang_error)
	      rospy.loginfo('throttle: %s, brake: %s, steer: %s', throttle, brake, steering)

        return throttle, brake, steering
