#!/usr/bin/env python
"""

"""

# # Standart libraries:
import rospy
from math import (
    sin,
    cos,
)
import numpy as np
from ast import (literal_eval)

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Float64MultiArray,
    Bool,
)
from geometry_msgs.msg import (Point)

# # Third party messages and services:
from gopher_ros_clearcore.srv import (LoggerControl)


class IONACOM():
    """
    
    """

    def __init__(
        self,
        node_name,
        right_arm_usage,
        right_arm_name,
        right_arm_y_mounting_angle,
        left_arm_usage,
        left_arm_name,
        left_arm_y_mounting_angle,
        chest_usage,
        stand_usage,
        stand_com,
    ):
        """
        
        """

        # # Private constants:
        self.__RIGHT_ARM_USAGE = right_arm_usage
        self.__RIGHT_ARM_NAME = right_arm_name
        self.__RIGHT_ARM_Y_MOUNTING_ANGLE = np.deg2rad(
            right_arm_y_mounting_angle
        )
        self.__LEFT_ARM_USAGE = left_arm_usage
        self.__LEFT_ARM_NAME = left_arm_name
        self.__LEFT_ARM_Y_MOUNTING_ANGLE = np.deg2rad(left_arm_y_mounting_angle)
        self.__CHEST_USAGE = chest_usage
        self.__CHEST_COM = np.array([chest_usage])
        self.__STAND_USAGE = stand_usage
        self.__STAND_COM = np.array([stand_com])

        # # Public constants:
        self.NODE_NAME = node_name

        # # Private variables:

        # # Public variables:
        self.__right_arm_com = [0.0, 0.0, 0.0]
        self.__left_arm_com = [0.0, 0.0, 0.0]
        self.__right_arm_mass = 0
        self.__left_arm_mass = 0
        self.__position_chest = 0

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        if self.__RIGHT_ARM_USAGE:
            self.__dependency_status['right_kinova_com'] = False

            self.__dependency_status_topics['right_kinova_com'] = (
                rospy.Subscriber(
                    f'/{self.__RIGHT_ARM_NAME}/kinova_com/is_initialized',
                    Bool,
                    self.__right_kinova_com_callback,
                )
            )

        if self.__LEFT_ARM_USAGE:
            self.__dependency_status['left_kinova_com'] = False

            self.__dependency_status_topics['left_kinova_com'] = (
                rospy.Subscriber(
                    f'/{self.__LEFT_ARM_NAME}/kinova_com/is_initialized',
                    Bool,
                    self.__left_kinova_com_callback,
                )
            )

        # TODO: Chest is_initialized.
        # if self.__CHEST_USAGE:
        #     self.__dependency_status['iona_chest'] = False

        # # Service provider:

        # # Service subscriber:
        self.__z_chest_logger = rospy.ServiceProxy(
            f'/z_chest_logger',
            LoggerControl,
        )

        # # Topic publisher:
        self.__com = rospy.Publisher(
            f'{self.NODE_NAME}/center_of_mass',
            Float64MultiArray,
            queue_size=1,
        )

        self.__com_bottom_chest_position = rospy.Publisher(
            f'{self.NODE_NAME}/center_of_mass/bottom_chest_position',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        if self.__RIGHT_ARM_USAGE:
            rospy.Subscriber(
                f'{self.__RIGHT_ARM_NAME}/center_of_mass',  #change
                Float64MultiArray,
                self.__right_arm_com_callback,
            )

        if self.__LEFT_ARM_USAGE:
            rospy.Subscriber(
                f'{self.__LEFT_ARM_NAME}/center_of_mass',  #change
                Float64MultiArray,
                self.__left_arm_com_callback,
            )
        if self.__CHEST_USAGE:
            rospy.Subscriber(
                '/chest_position',
                Point,
                self.__chest_position_callback,
            )

        # # Topic subscriber:

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __right_kinova_com_callback(self, message):
        """Monitors /right/kinova_come/is_initialized topic.

        """

        self.__dependency_status['right_kinova_com'] = message.data

    def __left_kinova_com_callback(self, message):
        """Monitors /left/kinova_come/is_initialized topic.

        """

        self.__dependency_status['left_kinova_com'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __right_arm_com_callback(self, message):
        """
        
        """

        data = message.data
        self.__right_arm_com = [data[0], data[1], data[2]]
        self.__right_arm_mass = data[3]

    def __left_arm_com_callback(self, message):
        """
        
        """

        data = message.data
        self.__left_arm_com = [data[0], data[1], data[2]]
        self.__left_arm_mass = data[3]

    def __chest_position_callback(self, message):
        """
        
        """

        self.__position_chest = message.z / 1000

    # Timer callbacks:

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
                self.__z_chest_logger(True)

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                self.__z_chest_logger(False)

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:
    def __calculate_iona_com(self):
        """
        
        """

        base_mass = 68

        if self.__STAND_USAGE:
            stand_mass = 39.107
        else:
            stand_mass = 0
        if self.__CHEST_USAGE:
            chest_mass = 17.316
        else:
            chest_mass = 0

        if not self.__RIGHT_ARM_USAGE:
            self.__right_arm_mass = 0

        if not self.__LEFT_ARM_USAGE:
            self.__left_arm_mass = 0

        # Mobile base height.
        base_height = 0.359

        # Z of center of mass of the chest (in Chest CS).
        chest_com_z = 0.386

        # Z of center of mass of the stand (in Stand CS).
        stand_com_z = self.__STAND_COM[2]

        # Transitions of the robot in the frame of a chest.
        x_right_arm_translation = 0.1375
        y_right_arm_translation = 0.139
        z_right_arm_translation = 0.1925

        # Transitions of the robot in the frame of a chest.
        x_left_arm_translation = 0.1375
        y_left_arm_translation = -0.139
        z_left_arm_translation = 0.1925

        # X position of the chest in Fetch CS.
        chest_offset = 0.0995

        base_stand_mass = base_mass + stand_mass

        # yapf: disable
        right_arm_com = np.array(
            [
                [self.__right_arm_com[0]],
                [self.__right_arm_com[1]],
                [self.__right_arm_com[2]],
                [1],
            ]
        )

        left_arm_com = np.array(
            [
                [self.__left_arm_com[0]],
                [self.__left_arm_com[1]],
                [self.__left_arm_com[2]],
                [1],
            ]
        )

        # Fetch mobile base COM in Fetch CS.
        base_com = np.array([[0.0024], [0], [0.180], [1]])

        # Stand COM in Fetch CS.
        stand_com = np.array(
            [[self.__STAND_COM[0]], [self.__STAND_COM[1]], [base_height + stand_com_z], [1]]
        )

        # Base and stand combined COM in Fetch CS.
        x_center_of_mass_combined = (
            base_com[0] * base_mass
            + stand_com[0] * stand_mass
        ) / (base_stand_mass)
        y_center_of_mass_combined = (
            base_com[1] * base_mass
            + stand_com[1] * stand_mass
        ) / (base_stand_mass)
        z_center_of_mass_combined = (
            base_com[2] * base_mass
            + stand_com[2] * stand_mass
        ) / (base_stand_mass)

        base_stand_com = np.array(
            [
                [x_center_of_mass_combined[0]],
                [y_center_of_mass_combined[0]],
                [z_center_of_mass_combined[0]],
                [1],
            ]
        )

        transformation_01 = np.array(
            [
                [1, 0, 0, -chest_offset],
                [0, 1, 0, 0],
                [0, 0, 1, (base_height + chest_com_z + self.__position_chest)],
                [0, 0, 0, 1],
            ]
        )

        transformation_01_no_chest = np.array(
            [
                [1, 0, 0, -chest_offset],
                [0, 1, 0, 0],
                [0, 0, 1, (base_height + chest_com_z)],
                [0, 0, 0, 1],
            ]
        )

        # Chest COM in Fetch CS.
        chest_com = np.dot(
            transformation_01, np.array([[self.__CHEST_COM[0]], [self.__CHEST_COM[1]], [self.__CHEST_COM[2]], [1]],)
        )

        chest_com_bottom_position = np.dot(
            transformation_01_no_chest,
            np.array([[0.08581], [0], [0.172112], [1]])
        )

        # Right arm:
        rotation_z_axis_right = np.array(
            [
                [0, 1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_x_axis_right = np.array(
            [
                [1, 0, 0, x_right_arm_translation],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_y_axis_right = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, y_right_arm_translation],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_z_axis_right = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, z_right_arm_translation],
                [0, 0, 0, 1],
            ]
        )
        rotation_y_axis_right = np.array(
            [
                [cos(self.__RIGHT_ARM_Y_MOUNTING_ANGLE), 0,
                 sin(self.__RIGHT_ARM_Y_MOUNTING_ANGLE), 0],
                [0, 1, 0, 0],
                [-sin(self.__RIGHT_ARM_Y_MOUNTING_ANGLE), 0,
                 cos(self.__RIGHT_ARM_Y_MOUNTING_ANGLE), 0],
                [0, 0, 0, 1],
            ]
        )

        right_arm_com = np.dot(
            np.dot(
                np.dot(
                    np.dot(
                        np.dot(
                            np.dot(transformation_01, rotation_z_axis_right),
                            translation_x_axis_right
                        ), translation_y_axis_right
                    ), translation_z_axis_right
                ), rotation_y_axis_right
            ), right_arm_com
        )

        # Left arm:
        rotation_z_axis_left = np.array(
            [
                [0, -1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_x_axis_left = np.array(
            [
                [1, 0, 0, x_left_arm_translation],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_y_axis_left = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, y_left_arm_translation],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        translation_z_axis_left = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, z_left_arm_translation],
                [0, 0, 0, 1],
            ]
        )
        rotation_y_axis_left = np.array(
            [
                [cos(self.__LEFT_ARM_Y_MOUNTING_ANGLE), 0,
                 sin(self.__LEFT_ARM_Y_MOUNTING_ANGLE), 0],
                [0, 1, 0, 0],
                [-sin(self.__LEFT_ARM_Y_MOUNTING_ANGLE), 0,
                 cos(self.__LEFT_ARM_Y_MOUNTING_ANGLE), 0],
                [0, 0, 0, 1],
            ]
        )

        left_arm_com = np.dot(
            np.dot(
                np.dot(
                    np.dot(
                        np.dot(
                            np.dot(transformation_01, rotation_z_axis_left),
                            translation_x_axis_left
                        ), translation_y_axis_left
                    ), translation_z_axis_left
                ), rotation_y_axis_left
            ), left_arm_com
        )

        # Total center of mass:
        x_center_of_mass_robot = (
            base_stand_com[0] * base_stand_mass + chest_com[0]
            * chest_mass + right_arm_com[0] * self.__right_arm_mass
            + left_arm_com[0] * self.__left_arm_mass
        ) / (
            self.__right_arm_mass + chest_mass + base_stand_mass
            + self.__left_arm_mass
        )
        y_center_of_mass_robot = (
            base_stand_com[1] * base_stand_mass + chest_com[1]
            * chest_mass + right_arm_com[1] * self.__right_arm_mass
            + left_arm_com[1] * self.__left_arm_mass
        ) / (
            self.__right_arm_mass + chest_mass + base_stand_mass
            + self.__left_arm_mass
        )
        z_center_of_mass_robot = (
            base_stand_com[2] * base_stand_mass + chest_com[2]
            * chest_mass + right_arm_com[2] * self.__right_arm_mass
            + left_arm_com[2] * self.__left_arm_mass
        ) / (
            self.__right_arm_mass + chest_mass + base_stand_mass
            + self.__left_arm_mass
        )

        robot_total_mass = (
            self.__right_arm_mass + chest_mass + base_stand_mass
        )
        center_of_mass_robot = [
            x_center_of_mass_robot[0],
            y_center_of_mass_robot[0],
            z_center_of_mass_robot[0],
            robot_total_mass,
        ]
        float64_array = Float64MultiArray()
        float64_array.data = center_of_mass_robot

        self.__com.publish(float64_array)

        # Center of mass (chest bottom position):
        z_center_of_mass_robot_chest_bottom_position = (
            base_stand_com[2] * base_stand_mass
            + chest_com_bottom_position[2] * chest_mass
            + right_arm_com[2] * self.__right_arm_mass
            + left_arm_com[2] * self.__left_arm_mass
        ) / (
            self.__right_arm_mass + chest_mass + base_stand_mass
            + self.__left_arm_mass
        )

        # yapf: enable

        chest_com_bottom = Float64MultiArray()
        chest_com_bottom.data = (z_center_of_mass_robot_chest_bottom_position)

        self.__com_bottom_chest_position.publish(chest_com_bottom)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__calculate_iona_com()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.
        self.__z_chest_logger(False)

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'iona_com',
        log_level=rospy.INFO,
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    right_arm_usage = rospy.get_param(
        param_name=f'{rospy.get_name()}/right_arm_usage',
        default=True,
    )

    right_arm_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/right_arm_name',
        default='my_gen3',
    )

    right_arm_y_mounting_angle = rospy.get_param(
        param_name=f'{rospy.get_name()}/right_arm_y_mounting_angle',
        default=45.0,
    )

    left_arm_usage = rospy.get_param(
        param_name=f'{rospy.get_name()}/left_arm_usage',
        default=False,
    )

    left_arm_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/left_arm_name',
        default='my_gen3',
    )

    left_arm_y_mounting_angle = rospy.get_param(
        param_name=f'{rospy.get_name()}/left_arm_y_mounting_angle',
        default=45.0,
    )

    chest_usage = rospy.get_param(
        param_name=f'{rospy.get_name()}/chest_usage',
        default=True,
    )

    chest_com = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/chest_com',
            default='[0.0858, 0, 0.1721]',
        )
    )

    stand_usage = rospy.get_param(
        param_name=f'{rospy.get_name()}/stand_usage',
        default=True,
    )

    stand_com = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/stand_com',
            default='[-0.0695, -0.0016, 0.5622]',
        )
    )

    class_instance = IONACOM(
        node_name=node_name,
        right_arm_usage=right_arm_usage,
        right_arm_name=right_arm_name,
        right_arm_y_mounting_angle=right_arm_y_mounting_angle,
        left_arm_usage=left_arm_usage,
        left_arm_name=left_arm_name,
        left_arm_y_mounting_angle=left_arm_y_mounting_angle,
        chest_usage=chest_usage,
        chest_com=chest_com,
        stand_usage=stand_usage,
        stand_com=stand_com,
    )

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()
