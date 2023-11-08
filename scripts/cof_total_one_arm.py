#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (
    Float64MultiArray,
    Bool,
)
from gopher_ros_clearcore.srv import (LoggerControl)
from geometry_msgs.msg import Point
from math import cos
from math import sin
import numpy as np


class cof_total():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:

        self.NODE_NAME = 'calc_com_total'
        self.__center_of_mass_right_arm = [0.0, 0.0, 0.0]
        self.__mass_right_arm = 0
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

        # # Service provider:

        # # Service subscriber:
        self.__z_chest_logger = rospy.ServiceProxy(
            f'/z_chest_logger',
            LoggerControl,
        )

        # # Topic publisher:
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/com_fin',
            Float64MultiArray,
            queue_size=1,
        )

        self.__no_chest = rospy.Publisher(
            f'{self.NODE_NAME}/com_fin_no_chest',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            'calc_com_arm_r/com_arm_r',  #change
            Float64MultiArray,
            self.__center_of_mass_right_arm_callback,
        )

        rospy.Subscriber(
            '/chest_position',
            Point,
            self.__chest_position_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __center_of_mass_right_arm_callback(self, message):
        data = message.data
        self.__center_of_mass_right_arm = [data[0], data[1], data[2]]
        self.__mass_right_arm  = data[3]

    def __chest_position_callback(self, message):
        self.__position_chest = message.z / 1000

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
    def calc_total(self):
        mass_base = 68
        mass_vertical = 39.107
        mass_base_combined = mass_base + mass_vertical
        mass_chest = 17.316
        height_base = 0.359
        height_chest = 0.386  # height of the origin of the cheat coordinate system
        height_vertical = 0.56219  # height of center of mass of a stand
        x_arm_translation = 0.1375  # x transition of the robot in the frame of a chest
        y_arm_translation = 0.139  # y transition of the robot in the frame of a chest
        z_arm_translation = 0.1925  # z transition of the robot in the frame of a chest
        arm_mounting_angle = 45 * np.pi / 180
        chest_offset = 0.0995

        center_of_mass_right_arm = np.array(
            [
                [self.__center_of_mass_right_arm[0]], [self.__center_of_mass_right_arm[1]],
                [self.__center_of_mass_right_arm[2]], [1]
            ]
        )
        center_of_mass_base = np.array([[0.0024], [0], [0.180], [1]])
        center_of_mass_vertical_part = np.array([[-0.06945], [-0.0016], [height_base + height_vertical], [1]])
        x_center_of_mass_combined = (center_of_mass_base[0] * mass_base
              + center_of_mass_vertical_part[0] * mass_vertical) / (mass_base_combined)
        y_center_of_mass_combined = (center_of_mass_base[1] * mass_base
              + center_of_mass_vertical_part[1] * mass_vertical) / (mass_base_combined)
        z_center_of_mass_combined = (center_of_mass_base[2] * mass_base
              + center_of_mass_vertical_part[2] * mass_vertical) / (mass_base_combined)
        Mass_pos_base = np.array([[x_center_of_mass_combined[0]], [y_center_of_mass_combined[0]], [z_center_of_mass_combined[0]], [1]])

        transformation_01 = np.array(
            [
                [1, 0, 0, -chest_offset], [0, 1, 0, 0],
                [0, 0, 1, (height_base + height_chest + self.__position_chest)], [0, 0, 0, 1]
            ]
        )

        transformation_01_no_chest = np.array(
            [
                [1, 0, 0, -chest_offset], [0, 1, 0, 0], [0, 0, 1, (height_base + height_chest)],
                [0, 0, 0, 1]
            ]
        )

        center_of_mass_chest = np.dot(transformation_01, np.array([[0.08581], [0], [0.172112], [1]]))

        center_of_mass_chest_bottom_position = np.dot(transformation_01_no_chest, np.array([[0.08581], [0], [0.172112], [1]]))

        rotation_z_axis = np.array(
            [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        translation_x_axis = np.array(
            [[1, 0, 0, x_arm_translation], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        translation_y_axis = np.array(
            [[1, 0, 0, 0], [0, 1, 0, y_arm_translation], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
        translation_z_axis = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z_arm_translation], [0, 0, 0, 1]]
        )
        rotation_y_axis = np.array(
            [
                [cos(arm_mounting_angle), 0, sin(arm_mounting_angle), 0], [0, 1, 0, 0],
                [-sin(arm_mounting_angle), 0, cos(arm_mounting_angle), 0], [0, 0, 0, 1]
            ]
        )

        center_of_mass_right_arm = np.dot(
            np.dot(
                np.dot(
                    np.dot(np.dot(np.dot(transformation_01, rotation_z_axis), translation_x_axis), translation_y_axis),
                    translation_z_axis
                ), rotation_y_axis
            ), center_of_mass_right_arm
        )

        x_center_of_mass_robot = (
            Mass_pos_base[0] * mass_base_combined + center_of_mass_chest[0] * mass_chest
            + center_of_mass_right_arm[0] * self.__mass_right_arm 
        ) / (self.__mass_right_arm  + mass_chest + mass_base_combined)
        y_center_of_mass_robot = (
            Mass_pos_base[1] * mass_base_combined + center_of_mass_chest[1] * mass_chest
            + center_of_mass_right_arm[1] * self.__mass_right_arm 
        ) / (self.__mass_right_arm  + mass_chest + mass_base_combined)
        z_center_of_mass_robot = (
            Mass_pos_base[2] * mass_base_combined + center_of_mass_chest[2] * mass_chest
            + center_of_mass_right_arm[2] * self.__mass_right_arm 
        ) / (self.__mass_right_arm  + mass_chest + mass_base_combined)

        z_center_of_mass_robot_chest_bottom_position = (
            Mass_pos_base[2] * mass_base_combined + center_of_mass_chest_bottom_position[2]
            * mass_chest + center_of_mass_right_arm[2] * self.__mass_right_arm 
        ) / (self.__mass_right_arm  + mass_chest + mass_base_combined)

        robot_total_mass = self.__mass_right_arm  + mass_chest + mass_base_combined
        center_of_mass_robot = [x_center_of_mass_robot[0], y_center_of_mass_robot[0], z_center_of_mass_robot[0], robot_total_mass]
        float64_array = Float64MultiArray()
        float64_array.data = center_of_mass_robot

        self.__publisher.publish(float64_array)

        center_of_mass_chest_bottom = Float64MultiArray()
        center_of_mass_chest_bottom.data = z_center_of_mass_robot_chest_bottom_position
        self.__no_chest.publish(center_of_mass_chest_bottom)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.calc_total()

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
    rospy.init_node('calc_com_total')

    class_instance = cof_total()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()