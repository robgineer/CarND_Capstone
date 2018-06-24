from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):

        min_speed = 0.1
        # get yaw controller object
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # define PID parameters
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.2

        # get throttle controller object
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # lowpass filter for velocity
        # required due to noisy inputs (cuts off outliers)
        cutoff = 0.5
        dt = 0.02
        self.velocity_lowpass = LowPassFilter(cutoff, dt)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.prev_timestamp = rospy.get_time()

    def control(self,
                curr_velocity,
                dbw_enabled,
                linear_velocity,
                angular_velocity):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # get filtered velocity values
        curr_velocity_filtered = self.velocity_lowpass.filt(curr_velocity)

        # get steering values
        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, curr_velocity_filtered)

        # store difference between predicted and actual values
        velocity_error = linear_velocity - curr_velocity_filtered

        # store current velocity
        self.previous_velocity = curr_velocity_filtered

        current_timestamp = rospy.get_time()
        cycle_time = current_timestamp - self.prev_timestamp
        # get throttle value
        throttle = self.throttle_controller.step(velocity_error, cycle_time)

        # set intial brake value to 0
        brake = 0

        # special case: car is standing. Set the brake value to a fixed no so car wont move
        if linear_velocity == 0 and curr_velocity_filtered < 0.1:
            throttle = 0
            brake = 400 # 700 for carla

            # car is braking. Decelerate comfortably
        elif throttle < 0.1 and velocity_error < 0:
            throttle = 0
            deceleration = max(velocity_error, self.decel_limit)
            brake = abs(deceleration) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering
