#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (
    Float64MultiArray,
    Bool,
    Float64,
)
from geometry_msgs.msg import Point
import math
from gopher_ros_clearcore.msg import Position
import numpy as np
from oculus_ros.msg import (
    ControllerButtons,
    ControllerJoystick,
)


class zmp_inv():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:
        self.NODE_NAME = 'zmp_inv'
        self.CONTROLLER_SIDE = 'left'
        self.MAX_CHEST_VELOCITY = 1.5
        self.__center_of_mass_robot = [0.001, 0.001, 0.001]
        self.__mass_rob = 130
        self.__position_chest = 0
        self.__steady_position = 440
        self.__absolute_chest_position = self.__position_chest
        self.__current_linear_acceleration = 0
        self.__current_rotational_acceleration = 0
        self.__current_rotational_velocity = 0
        self.__p_gain = 120
        self.__oculus_joystick = ControllerJoystick()
        self.__joystick_button_state = 0
        self.__control_mode = 'autonomy_control'
        self.__z_coordinate_center_of_mass_chest_bottom = 0.5
        self.__center_of_mass_distance_to_origin = 0.001
        self.__max_linear_acceleration = 0.5
        self.__max_rotation_acceleration = 30
        self.__max_rotation_velocity = 60

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            # 'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            # 'dependency_node_name':
            #     rospy.Subscriber(
            #         f'/dependency_node_name/is_initialized',
            #         Bool,
            #         self.__dependency_name_callback,
            #     ),
        }

        # # Topic publisher:
        self.__error = rospy.Publisher(
            '/acceleration/error',
            Float64,
            queue_size=1,
        )

        self.__max_motion_parameters = rospy.Publisher(
            '/fetch/max_motion_parameters',
            Float64MultiArray,
            queue_size=1,
        )

        self.__publisher_pos_abs = rospy.Publisher(
            '/z_chest_pos',
            Position,
            queue_size=1,
        )

        # # Topic subscriber:

        rospy.Subscriber(
            'calc_com_total/com_fin',  #change
            Float64MultiArray,
            self.__center_of_mass_robot_callback,
        )

        rospy.Subscriber(
            '/chest_position',
            Point,
            self.__Chest_Pos_callback,
        )

        rospy.Subscriber(
            '/fetch/current_motion_parameters',
            Float64MultiArray,
            self.__current_motion_parameters_callback,
        )

        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )

        rospy.Subscriber(
            '/calc_com_total/com_fin_no_chest',  #change
            Float64MultiArray,
            self.__z_coordinate_chest_bottom_position_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __z_coordinate_chest_bottom_position_callback(self, message):
        self.__z_coordinate_center_of_mass_chest_bottom = message.data[0]

    def __center_of_mass_robot_callback(self, message):
        self.__center_of_mass_robot = [
            message.data[0], message.data[1], message.data[2]
        ]
        self.__mass_rob = message.data[3]

    def __Chest_Pos_callback(self, message):
        self.__position_chest = message.z

    def __current_motion_parameters_callback(self, message):
        self.__current_linear_acceleration = message.data[0]
        self.__current_rotational_acceleration = message.data[1]
        self.__current_rotational_velocity = message.data[3]

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:
    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__control_mode = 'user_control'
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
            self.__control_mode = 'autonomy_control'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 3
        ):
            self.__joystick_button_state = 0

    def max_acc_calc(
        self,
        current_linear_acceleration,
    ):

        e = 2.7
        g = 9.81
        x_limit_coordinate = -0.221 / e
        y_limit_coordinate = 0.221 / e
        acc_centrifugal_max = 0
        acc_tangential_max = 0

        self.__center_of_mass_distance_to_origin = math.sqrt(
            self.__center_of_mass_robot[0] * self.__center_of_mass_robot[0]
            + self.__center_of_mass_robot[1] * self.__center_of_mass_robot[1]
        )

        cosa = self.__center_of_mass_robot[
            0] / self.__center_of_mass_distance_to_origin
        sina = self.__center_of_mass_robot[
            1] / self.__center_of_mass_distance_to_origin
        tga = self.__center_of_mass_robot[1] / self.__center_of_mass_robot[0]

        acc_x_axis_max = (
            x_limit_coordinate * self.__mass_rob * g
            - self.__mass_rob * self.__center_of_mass_robot[0] * g
        ) / (
            -self.__mass_rob * self.__z_coordinate_center_of_mass_chest_bottom
        )

        acc_y_axis_max = (
            y_limit_coordinate * self.__mass_rob * g
            - self.__mass_rob * self.__center_of_mass_robot[1] * g
        ) / (
            -self.__mass_rob * self.__z_coordinate_center_of_mass_chest_bottom
        )

        # Chanage the sighn of centrifugal acceleration in x component
        # acc_centrifugal_max = (acc_x_axis_max - tga * acc_y_axis_max
        #            - current_linear_acceleration) / (cosa - tga * sina)

        # No centrifugal acc tangential calculation
        # max_rotation_velocity=0
        # acc_tangential_max=math.sqrt(pow((acc_x_axis_max-current_linear_acceleration), 2)+pow(acc_y_axis_max, 2))

        # Original
        acc_centrifugal_max = (
            acc_x_axis_max - tga * acc_y_axis_max - current_linear_acceleration
        ) / (-cosa - tga * sina)

        # Tangential calculation
        acc_tangential_max = (acc_y_axis_max
                              - sina * acc_centrifugal_max) / (cosa)
        acc_centrifugal_max = abs(acc_centrifugal_max)
        acc_tangential_max = abs(acc_tangential_max)
        self.__max_rotation_velocity = math.sqrt(
            acc_centrifugal_max / self.__center_of_mass_distance_to_origin
        )

        self.__max_rotation_acceleration = acc_tangential_max / self.__center_of_mass_distance_to_origin
        self.__max_linear_acceleration = 0.8 * acc_x_axis_max

        float64_array = Float64MultiArray()
        float64_array.data = [
            self.__max_linear_acceleration, self.__max_rotation_acceleration,
            self.__max_rotation_velocity
        ]

        # print(
        #     [
        #         self.__max_linear_acceleration,
        #         1.5 * self.__max_rotation_acceleration,
        #     ]
        # )

        self.__max_motion_parameters.publish(float64_array)

        #print(max_linea_acc, max_rot_acc)

    def chest_pos_calc(
        self, current_linear_acceleration, current_rotational_velocity,
        current_rotational_acceleration
    ):
        e = 2.7
        g = 9.81
        x_limit_coordinate = -0.221 / e
        y_limit_coordinate = 0.221 / e
        acc_centrifugal_current = 0
        current_rotational_velocity = abs(current_rotational_velocity)

        cosa = self.__center_of_mass_robot[
            0] / self.__center_of_mass_distance_to_origin
        sina = self.__center_of_mass_robot[
            1] / self.__center_of_mass_distance_to_origin

        acc_x_axis_max_current = (
            x_limit_coordinate * self.__mass_rob * g
            - self.__mass_rob * self.__center_of_mass_robot[0] * g
        ) / (-self.__mass_rob * self.__center_of_mass_robot[2])

        # if current_rotational_velocity < 0.01:
        #     current_rotational_velocity = 0
        # if current_rotational_acceleration <= (
        #     0.21 * self.__max_rotation_velocity
        # ):
        #     current_rotational_acceleration = 0

        acc_centrifugal_current = -self.__center_of_mass_distance_to_origin * current_rotational_velocity * current_rotational_velocity
        acc_tangential_current = -self.__center_of_mass_distance_to_origin * current_rotational_acceleration
        acc_x_axis_current = -acc_centrifugal_current * cosa + acc_tangential_current * sina + current_linear_acceleration  # - centr may change to +

        #Position controller
        e_acc = acc_x_axis_max_current - acc_x_axis_current
        self.__absolute_chest_position = self.__position_chest + e_acc * self.__p_gain

        message = Float64()
        message.data = e_acc
        self.__error.publish(message)

        if self.__absolute_chest_position >= 440:
            self.__absolute_chest_position = 440
        elif self.__absolute_chest_position <= 20:
            self.__absolute_chest_position = 20
        # print(
        #     [
        #         acc_centrifugal_current, acc_tangential_current,
        #         current_linear_acceleration
        #     ]
        # )
        #print([acc_x_axis_max_current, acc_x_axis_current])
        #print(self.__absolute_chest_position)

    def __publish_pos(self):

        abs_pos_msg = Position()
        abs_pos_msg.velocity = self.MAX_CHEST_VELOCITY

        self.__joystick_button_state_machine()

        if self.__control_mode != 'user_control':
            abs_pos_msg.position = self.__absolute_chest_position
            self.__publisher_pos_abs.publish(abs_pos_msg)

        # print(np.array([
        #     abs_pos_msg.position,
        #     abs_pos_msg.velocity,
        # ]).round(3))

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.max_acc_calc(self.__current_linear_acceleration)

        self.chest_pos_calc(
            self.__current_linear_acceleration,
            self.__current_rotational_velocity,
            self.__current_rotational_acceleration,
        )

        self.__publish_pos()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node('zmp_inv')

    class_instance = zmp_inv()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()