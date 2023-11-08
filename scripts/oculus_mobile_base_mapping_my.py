#!/usr/bin/env python
"""

"""

import rospy
import math
import numpy as np

from geometry_msgs.msg import (Twist)
from std_msgs.msg import (Float64MultiArray)

from oculus_ros.msg import (
    ControllerJoystick,
    ControllerButtons,
)


class OculusMobileBaseMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='left',
        max_linear_speed=0.5,  # Meters/second.
        max_rotation_speed=60,  # Degrees/seconds.
        max_linear_acceleration=5,  # Meters/second^2.
        max_rotation_acceleration=1800,  # Degrees/second^2.
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ROTATION_SPEED = math.radians(max_rotation_speed)

        self.MIN_LINEAR_ACCELERATION = 0.1
        self.MAX_LINEAR_ACCELERATION = max_linear_acceleration

        self.MIN_ROTATION_ACCELERATION = (
            0.2 * math.radians(max_rotation_acceleration)
        )
        self.MAX_ROTATION_ACCELERATION = math.radians(max_rotation_acceleration)

        # # Private variables:
        self.__oculus_joystick = ControllerJoystick()

        # Set target velocities.
        self.__target_linear_velocity = 0.0
        self.__target_rotation_velocity = 0.0
        self.__scale_linear_speed=5
        self.__scale_rotation_speed=5
        self.__scale_linear_reverse=0.5
        self.__scale_rotation_reverse=1

        # Calculated accelerations based on the joystick input.
        self.__current_linear_acceleration = self.MIN_LINEAR_ACCELERATION
        self.__current_rotation_acceleration = self.MAX_ROTATION_ACCELERATION

        # Calculated velocities based on the acceleration algorithm.
        self.__current_linear_velocity = 0.0
        self.__current_rotation_velocity = 0.0

        self.__previous_time = rospy.get_time()

        self.__joystick_button_state = 0
        self.__control_mode = 'full'

        # Test zone:

        # # Public variables:

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__mobilebase_twist_velocity = rospy.Publisher(
            '/base_controller/command',
            Twist,
            queue_size=1,
        )

        self.__mobilebase_twist_acceleration = rospy.Publisher(
            '/current_acc',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        
        rospy.Subscriber(
            '/zmp_acc_max',
            Float64MultiArray,
            self.__zmp_callback,
        )
        # # Timer:
        rospy.Timer(
            rospy.Duration(1.0 / 10.0),
            self.__set_target_velocities_accelerations_timer,
        )

       
        

    # # Service handlers:

    # # Topic callbacks:
    def __zmp_callback(self, message):
        self.MAX_LINEAR_ACCELERATION=message.data[0]
        self.MAX_ROTATION_ACCELERATION=message.data[1]
        self.MAX_ROTATION_SPEED=message.data[2]

    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    ## Timer functions:
    def __set_target_velocities_accelerations_timer(self, event):
        """
        
        """

        self.__set_target_velocities_accelerations()

    # # Private methods:
    def __check_dead_zones(self):
        """
        
        """

        updated_joystick = np.array(
            [
                self.__oculus_joystick.position_x,
                self.__oculus_joystick.position_y,
            ]
        )

        dead_zone_margins = {
            'up': 15,
            'down': 25,
            'left_right': 0,
        }

        angle = math.degrees(
            math.atan2(updated_joystick[1], updated_joystick[0])
        )

        if (
            (
                angle < 90 + dead_zone_margins['up']
                and angle > 90 - dead_zone_margins['up']
            ) or (
                angle > -90 - dead_zone_margins['down']
                and angle < -90 + dead_zone_margins['down']
            )
        ):
            updated_joystick[0] = 0.0

        elif (
            (
                angle < 0 + dead_zone_margins['left_right']
                and angle > 0 - dead_zone_margins['down']
            ) or (
                angle < -180 + dead_zone_margins['left_right']
                and angle > 180 - dead_zone_margins['down']
            )
        ):
            updated_joystick[1] = 0.0

        return updated_joystick

    def __set_target_velocities_accelerations(self):
        """
        
        """

        updated_joystick = self.__check_dead_zones()

        self.__target_linear_velocity = 0
        self.__target_rotation_velocity = 0

        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            # Linear velocity.
            self.__target_linear_velocity = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                [-self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED],
            )

            if self.__target_linear_velocity <= -0.5 * self.MAX_LINEAR_SPEED:
                self.__target_linear_velocity = -0.5 * self.MAX_LINEAR_SPEED

        if abs(self.__oculus_joystick.position_x) > 0.01:  # Noisy joystick.
            # Rotation velocity.
            self.__target_rotation_velocity = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [self.MAX_ROTATION_SPEED, -self.MAX_ROTATION_SPEED],
            )

    def __update_velocity(
        self,
        target_velocity,
        current_velocity,
        acceleration,
    ):
        """
        
        """

        # Get updated time step.
        current_time = rospy.get_time()
        time_step = current_time - self.__previous_time

        self.__previous_time = current_time

        if current_velocity > target_velocity:
            current_velocity = current_velocity - acceleration * time_step

        elif current_velocity < target_velocity:
            current_velocity = current_velocity + acceleration * time_step

        elif abs(current_velocity - target_velocity) <= 0.01:
            current_velocity = target_velocity

        return current_velocity

    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__control_mode = 'rotation'
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 2

        # State 2: Joystick button was pressed. Normal control mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 2
        ):
            self.__control_mode = 'full'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 3
        ):
            self.__joystick_button_state = 0

    def __acc_calc(self, current_velocity, max_speed, max_acc, min_acc, scale_speed, scale_reverse):

        if current_velocity > 0:
            current_acc = np.interp(
                abs(current_velocity),
                [
                    0.0,
                   max_speed * scale_speed,
                ],
                [
                    min_acc,
                    max_acc,
                ],
            )
        elif current_velocity < 0:
            current_acc = np.interp(
                abs(current_velocity),
                [
                    0,
                    scale_reverse * max_speed * scale_speed,
                ],
                [
                    min_acc,
                    scale_reverse * max_acc,
                ],
            )
        else:
            current_acc = min_acc
        
        return current_acc


    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__current_linear_velocity = self.__update_velocity(
            self.__target_linear_velocity,
            self.__current_linear_velocity,
            self.__current_linear_acceleration,
        )
        self.__current_rotation_velocity = self.__update_velocity(
            self.__target_rotation_velocity,
            self.__current_rotation_velocity,
            self.__current_rotation_acceleration,
        )
        
        self.__current_linear_acceleration=self.__acc_calc(
            self.__current_linear_velocity,
            self.MAX_LINEAR_SPEED,
            0.8*self.MAX_LINEAR_ACCELERATION,
            self.MIN_LINEAR_ACCELERATION,
            self.__scale_linear_speed,
            self.__scale_linear_reverse,
        )

        self.__current_rotation_acceleration=self.__acc_calc(
            self.__current_rotation_velocity,
            self.MAX_ROTATION_SPEED,
            self.MAX_ROTATION_ACCELERATION,
            self.MIN_ROTATION_ACCELERATION,
            self.__scale_rotation_speed,
            self.__scale_rotation_reverse,
        )

        self.publish_twist_velocity()
        self.publish_accelerations()

        # print(
        #     np.array(
        #         [
        #             self.__current_rotation_velocity,
        #             self.__current_rotation_acceleration,
        #         ]
        #     ).round(2)
        # )

    def publish_twist_velocity(self):
        """
        
        """

        twist_message = Twist()
        twist_message.linear.x = self.__current_linear_velocity
        twist_message.angular.z = self.__current_rotation_velocity

        self.__joystick_button_state_machine()

        if self.__control_mode == 'rotation':
            twist_message.linear.x = 0.0

        self.__mobilebase_twist_velocity.publish(twist_message)

    def publish_accelerations(self):
        """
        
        """
        acc_message = Float64MultiArray()
        acc_message.data = [self.__current_linear_acceleration, self.__current_rotation_acceleration, self.__target_linear_velocity, self.__target_rotation_velocity]

        self.__mobilebase_twist_acceleration.publish(acc_message)

def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop mobile base motion.

    print('\nNode is shut down.\n')


def main():
    """
    
    """

    # # ROS node:
    rospy.init_node('oculus_mobile_base_mapping')
    rospy.on_shutdown(node_shutdown)

    mobile_base_mapping = OculusMobileBaseMapping(
        controller_side='right',
        max_linear_speed=0.5,
    )

    print('\nOculus-mobile base mapping is ready.\n')

    while not rospy.is_shutdown():
        mobile_base_mapping.main_loop()


if __name__ == '__main__':
    main()
