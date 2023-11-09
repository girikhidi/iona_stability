#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from sensor_msgs.msg import JointState
from math import cos
from math import sin
import numpy as np


class com_arm_calc():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:

        self.NODE_NAME = 'calc_com_arm_r'
        self.__center_of_mass_arm = [0.0, 0.0, 0.0, 0.0]
        self.__joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ROBOTNAME = '/my_gen3'

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
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/com_arm_r',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.ROBOTNAME}/base_feedback/joint_state',  #change
            JointState,
            self.__joint_values_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __joint_values_callback(self, message):
        self.__joint_values = message.position

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
    def cacl_com(self):
        mass_base = 1.697
        mass_link_1 = 1.377
        mass_link_2 = 1.1636
        mass_link_3 = 1.1636
        mass_link_4 = 0.930
        mass_link_5 = 0.678
        mass_link_6 = 0.678
        mass_link_7 = 0.500
        mass_load = 7
        mass_gripper = 0.925

        joint_values = self.__joint_values

        matrix_01 = np.array(
            [
                [cos(joint_values[0]), -sin(joint_values[0]), 0, 0],
                [-sin(joint_values[0]), -cos(joint_values[0]), 0, 0],
                [0, 0, -1, 0.1564], [0, 0, 0, 1]
            ]
        )
        matrix_12 = np.array(
            [
                [cos(joint_values[1]), -sin(joint_values[1]), 0, 0],
                [0, 0, -1, 0.0054],
                [sin(joint_values[1]),
                 cos(joint_values[1]), 0, -0.1284], [0, 0, 0, 1]
            ]
        )
        matrix_23 = np.array(
            [
                [cos(joint_values[2]), -sin(joint_values[2]), 0, 0],
                [0, 0, 1, -0.2104],
                [-sin(joint_values[2]), -cos(joint_values[2]), 0, -0.0064],
                [0, 0, 0, 1]
            ]
        )
        matrix_34 = np.array(
            [
                [cos(joint_values[3]), -sin(joint_values[3]), 0, 0],
                [0, 0, -1, -0.0064],
                [sin(joint_values[3]),
                 cos(joint_values[3]), 0, -0.2104], [0, 0, 0, 1]
            ]
        )
        matrix_45 = np.array(
            [
                [cos(joint_values[4]), -sin(joint_values[4]), 0, 0],
                [0, 0, 1, -0.2084],
                [-sin(joint_values[4]), -cos(joint_values[4]), 0, -0.0064],
                [0, 0, 0, 1]
            ]
        )
        matrix_56 = np.array(
            [
                [cos(joint_values[5]), -sin(joint_values[5]), 0, 0],
                [0, 0, -1, 0],
                [sin(joint_values[5]),
                 cos(joint_values[5]), 0, -0.1059], [0, 0, 0, 1]
            ]
        )
        matrix_67 = np.array(
            [
                [cos(joint_values[6]), -sin(joint_values[6]), 0, 0],
                [0, 0, 1, -0.1059],
                [-sin(joint_values[6]), -cos(joint_values[6]), 0, 0],
                [0, 0, 0, 1]
            ]
        )
        matrix_7L = np.array(
            [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, -0.0615], [0, 0, 0, 1]]
        )

        center_of_mass_gripper = np.array([0, 0, 0.058])
        center_of_mass_load = np.array([0, 0, 0.08765])

        center_of_mass_base = np.array([-0.000648, -0.000166, 0.084487])
        center_of_mass_link_1 = np.dot(
            matrix_01,
            np.array([-0.000023, -0.010364, -0.073360, 1]).transpose()
        )
        center_of_mass_link_2 = np.dot(
            np.dot(matrix_01, matrix_12),
            np.array([-0.000044, -0.099580, -0.013278, 1]).transpose()
        )
        center_of_mass_link_3 = np.dot(
            np.dot(np.dot(matrix_01, matrix_12), matrix_23),
            np.array([-0.000044, -0.006641, -0.117892, 1]).transpose()
        )
        center_of_mass_link_4 = np.dot(
            np.dot(np.dot(np.dot(matrix_01, matrix_12), matrix_23), matrix_34),
            np.array([-0.000018, -0.075478, -0.015006, 1]).transpose()
        )
        center_of_mass_link_5 = np.dot(
            np.dot(
                np.dot(
                    np.dot(np.dot(matrix_01, matrix_12), matrix_23), matrix_34
                ), matrix_45
            ),
            np.array([0.000001, -0.009432, -0.063883, 1]).transpose()
        )
        center_of_mass_link_6 = np.dot(
            np.dot(
                np.dot(
                    np.dot(
                        np.dot(np.dot(matrix_01, matrix_12), matrix_23),
                        matrix_34
                    ), matrix_45
                ), matrix_56
            ),
            np.array([0.000001, -0.045483, -0.009650, 1]).transpose()
        )
        center_of_mass_link_7 = np.dot(
            np.dot(
                np.dot(
                    np.dot(
                        np.dot(
                            np.dot(np.dot(matrix_01, matrix_12), matrix_23),
                            matrix_34
                        ), matrix_45
                    ), matrix_56
                ), matrix_67
            ),
            np.array([-0.000281, -0.011402, -0.029798, 1]).transpose()
        )

        transformation_matrix_load = np.dot(
            np.dot(
                np.dot(
                    np.dot(
                        np.dot(
                            np.dot(np.dot(matrix_01, matrix_12), matrix_23),
                            matrix_34
                        ), matrix_45
                    ), matrix_56
                ), matrix_67
            ), matrix_7L
        )

        mass_load_combined = mass_load + mass_gripper
        center_of_mass_load_combined = np.array(
            [
                0, 0,
                (
                    center_of_mass_gripper[2] * mass_gripper
                    + center_of_mass_load[2] * mass_load
                ) / mass_load_combined, 1
            ]
        )
        center_of_mass_load_combined = np.dot(
            transformation_matrix_load, center_of_mass_load_combined.transpose()
        )

        x_center_of_mass_arm = (
            center_of_mass_base[0] * mass_base + center_of_mass_link_1[0]
            * mass_link_1 + center_of_mass_link_2[0] * mass_link_2
            + center_of_mass_link_3[0] * mass_link_3 + center_of_mass_link_4[0]
            * mass_link_4 + center_of_mass_link_5[0] * mass_link_5
            + center_of_mass_link_6[0] * mass_link_6 + center_of_mass_link_7[0]
            * mass_link_7 + center_of_mass_load_combined[0] * mass_load_combined
        ) / (
            mass_base + mass_link_1 + mass_link_2 + mass_link_3 + mass_link_4
            + mass_link_5 + mass_link_6 + mass_link_7 + mass_load_combined
        )

        y_center_of_mass_arm = (
            center_of_mass_base[1] * mass_base + center_of_mass_link_1[1]
            * mass_link_1 + center_of_mass_link_2[1] * mass_link_2
            + center_of_mass_link_3[1] * mass_link_3 + center_of_mass_link_4[1]
            * mass_link_4 + center_of_mass_link_5[1] * mass_link_5
            + center_of_mass_link_6[1] * mass_link_6 + center_of_mass_link_7[1]
            * mass_link_7 + center_of_mass_load_combined[1] * mass_load_combined
        ) / (
            mass_base + mass_link_1 + mass_link_2 + mass_link_3 + mass_link_4
            + mass_link_5 + mass_link_6 + mass_link_7 + mass_load_combined
        )

        z_center_of_mass_arm = (
            center_of_mass_base[2] * mass_base + center_of_mass_link_1[2]
            * mass_link_1 + center_of_mass_link_2[2] * mass_link_2
            + center_of_mass_link_3[2] * mass_link_3 + center_of_mass_link_4[2]
            * mass_link_4 + center_of_mass_link_5[2] * mass_link_5
            + center_of_mass_link_6[2] * mass_link_6 + center_of_mass_link_7[2]
            * mass_link_7 + center_of_mass_load_combined[2] * mass_load_combined
        ) / (
            mass_base + mass_link_1 + mass_link_2 + mass_link_3 + mass_link_4
            + mass_link_5 + mass_link_6 + mass_link_7 + mass_load_combined
        )

        mass_total_arm = mass_base + mass_link_1 + mass_link_2 + mass_link_3 + mass_link_4 + mass_link_5 + mass_link_6 + mass_link_7 + mass_load_combined
        self.__center_of_mass_arm = [
            x_center_of_mass_arm, y_center_of_mass_arm, z_center_of_mass_arm,
            mass_total_arm
        ]

        float64_array = Float64MultiArray()
        float64_array.data = self.__center_of_mass_arm
        self.__publisher.publish(float64_array)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.cacl_com()

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
    rospy.init_node('calc_com_arm_r')

    class_instance = com_arm_calc()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()
