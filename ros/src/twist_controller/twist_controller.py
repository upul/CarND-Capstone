from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
                 max_steer_angle, *args, **kwargs):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed,
                                            max_lat_accel, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        rospy.loginfo('steering angle: {}'.format(steer))
        return 0.5, 0.0, steer*5.0
