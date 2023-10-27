#!/usr/bin/env python
"""

"""

import rospy
import math
import numpy as np

from geometry_msgs.msg import (Twist)
from std_msgs.msg import (Float64MultiArray)

from oculus_ros.msg import (
    ControllerButtons,
    ControllerJoystick,
)


class OculusMobileBaseMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='left', 
        max_linear_speed=0.5,               # Meters/second.
        max_rotation_speed=60,              # Degrees/seconds.
        max_linear_acceleration=5,          # Meters/second^2.
        max_rotation_acceleration=1800,     # Degrees/second^2.
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ROTATION_SPEED = math.radians(max_rotation_speed)
        self.MAX_LINEAR_ACCELERATION = max_linear_acceleration
        self.MAX_ROTATION_ACCELERATION = math.radians(max_rotation_acceleration)

        # # Private variables:
        self.__oculus_joystick = ControllerJoystick()
        self.__oculus_buttons = ControllerButtons()

        # Set target velocities.
        self.__target_linear_velocity = 0.0
        self.__target_rotation_velocity = 0.0
        self.__target_linear_acc = 0.0
        self.__target_rotation_acc = 0.0
        
        # Calculated accelerations based on the joystick input.
        self.__current_linear_acceleration = 0.01 * self.MAX_LINEAR_ACCELERATION
        self.__current_rotation_acceleration = 0.01 * self.MAX_ROTATION_ACCELERATION

        # Calculated velocities based on the acceleration algorithm.
        self.__current_linear_velocity = 0.0
        self.__current_rotation_velocity = 0.0

        self.__previous_time = rospy.get_time()
        self.__previous_time_linear=rospy.get_time()

        self.__joystick_button_state = 0
        self.__control_mode = 'full'

        # Test zone:
        self.__flag_flag = False

        # # Public variables:

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__mobilebase_twist = rospy.Publisher(
            'base_controller/command',
            Twist,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

        rospy.Subscriber(
            '/zmp_acc_max',
            Float64MultiArray,
            self.__zmp_callback,
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

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

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

        self.__target_rotation_acc=self.MAX_ROTATION_ACCELERATION
        self.__target_linear_acc=self.MAX_LINEAR_ACCELERATION
        self.__target_linear_velocity=0
        self.__target_rotation_velocity=0

        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            # Linear velocity.
            self.__target_linear_velocity = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                [-self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED],
            )

            # Linear acceleration.
            # if self.__oculus_joystick.position_y >0: 
            #     self.__target_linear_acc = np.interp(
            #         round(updated_joystick[1], 4),
            #         [0.01, 1.0],
            #         # NOTE: 0.08 and 0.8 - experimental.
            #         [0.08 * self.MAX_LINEAR_ACCELERATION, 0.8 * self.MAX_LINEAR_ACCELERATION], 
            #         )
            # elif self.__oculus_joystick.position_y <0: 
            #     self.__target_linear_acc = np.interp(
            #         round(updated_joystick[1], 4),
            #         [-1.0, -0.01],
            #         # NOTE: 0.08 and 0.8 - experimental.
            #         [-0.8 * self.MAX_LINEAR_ACCELERATION, -0.08 * self.MAX_LINEAR_ACCELERATION], 
            #         ) 
            self.__target_linear_acc = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                # NOTE: 0.08 and 0.8 - experimental.
                [-0.8 * self.MAX_LINEAR_ACCELERATION, 0.8 * self.MAX_LINEAR_ACCELERATION], 
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

            # Rotation acceleration.
            # if self.__oculus_joystick.position_x >0:
            #     self.__target_rotation_acc = np.interp(
            #         round(updated_joystick[0], 4),
            #         [0.01, 1.0],
            #         [0.01 * self.MAX_ROTATION_ACCELERATION, self.MAX_ROTATION_ACCELERATION], 
            #     ) 
            # if self.__oculus_joystick.position_x <0:
            #     self.__target_rotation_acc = np.interp(
            #         round(updated_joystick[0], 4),
            #         [-1.0, -0.01],
            #         [-self.MAX_ROTATION_ACCELERATION, -0.01*self.MAX_ROTATION_ACCELERATION], 
            #     ) 

            
            self.__target_rotation_acc = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [-self.MAX_ROTATION_ACCELERATION, self.MAX_ROTATION_ACCELERATION], 
                )
            

        # Decceleration with 0 joystick input:
        acc_acc = 2.0

        self.__current_linear_acceleration=self.__update_acc(
            self.__target_linear_acc,
            self.__current_linear_acceleration,
            acc_acc,
            )
        self.__current_rotation_acceleration=self.__update_acc(
            self.__target_rotation_acc,
            self.__current_rotation_acceleration,
            acc_acc
        )
        


    def __update_acc(
        self,
        target_acc,
        current_acc,
        acc_acc,
    ):
        """
        
        """

        # Get updated time step.
        current_time = rospy.get_time()
        time_step = current_time - self.__previous_time_linear

        self.__previous_time = current_time

        # Forward motion.
        if target_acc > 0:
            # Target velocity reached.
            if current_acc >= target_acc:
                current_acc = target_acc

            # Acceleration.
            else:
                current_acc = current_acc + acc_acc * time_step

        # Backward motion.
        elif target_acc < 0:
            # Target velocity reached.
            if current_acc <= target_acc:
                current_acc = target_acc

            # Acceleration.
            else:
                current_acc = current_acc - acc_acc * time_step

        # Stopping.
        elif target_acc == 0:
            # Target velocity reached.
            if abs(current_acc - target_acc) < 0.05:
                current_acc = 0.0

            # Decceleration.
            else:
                if current_acc > 0:
                    current_acc = (
                        current_acc - acc_acc * time_step
                    )

                elif current_acc < 0:
                    current_acc = (
                        current_acc + acc_acc * time_step
                    )

        return current_acc

    def __update_velocity(
        self,
        target_velocity,
        current_velocity,
        acceleration,
    ):
        """
        
        """
        acceleration=abs(acceleration)
        # Get updated time step.
        current_time = rospy.get_time()
        time_step = current_time - self.__previous_time

        self.__previous_time = current_time

        # Forward motion.
        if target_velocity > 0:
            # Target velocity reached.
            if current_velocity >= target_velocity:
                current_velocity = target_velocity

            # Acceleration.
            else:
                current_velocity = current_velocity + acceleration * time_step

        # Backward motion.
        elif target_velocity < 0:
            # Target velocity reached.
            if current_velocity <= target_velocity:
                current_velocity = target_velocity

            # Acceleration.
            else:
                current_velocity = current_velocity - acceleration * time_step

        # Stopping.
        elif target_velocity == 0:
            # Target velocity reached.
            if abs(current_velocity - target_velocity) < 0.05:
                current_velocity = 0.0

            # Decceleration.
            else:
                if current_velocity > 0:
                    current_velocity = (
                        current_velocity - acceleration * time_step
                    )

                elif current_velocity < 0:
                    current_velocity = (
                        current_velocity + acceleration * time_step
                    )

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

    # # Public methods:
    def main_loop(self):
        """
        
        """

        # self.__set_target_velocities()
        self.__set_target_velocities_accelerations()

        self.__current_linear_velocity = self.__update_velocity(
            self.__target_linear_velocity,
            self.__current_linear_velocity,
            # self.LINEAR_ACCELERATION,
            self.__current_linear_acceleration,
        )
        self.__current_rotation_velocity = self.__update_velocity(
            self.__target_rotation_velocity,
            self.__current_rotation_velocity,
            # self.ROTATION_ACCELERATION,
            self.__current_rotation_acceleration,
        )

        self.publish_twist_command()
        # print(np.array([self.__current_linear_velocity, self.__current_linear_acceleration]).round(2))
        # print(np.array([self.__current_rotation_velocity, self.__current_rotation_acceleration]).round(2))


    def publish_twist_command(self):
        """
        
        """

        twist_message = Twist()
        twist_message.linear.x = self.__current_linear_velocity
        twist_message.angular.z = self.__current_rotation_velocity

        self.__joystick_button_state_machine()

        if self.__control_mode == 'rotation':
            twist_message.linear.x = 0.0

        self.__mobilebase_twist.publish(twist_message)
        


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
