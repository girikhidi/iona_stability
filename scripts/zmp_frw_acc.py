#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
import math


class zmp_forw():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:
        self.NODE_NAME = 'zmp_forw_acc'
        self.MAX_LINEAR_SPEED = 0.5
        self.__robot_total_mass = [0.001, 0.001, 0.001]
        self.__total_mass_robot = 130
        self.__current_linear_acceleration = 0

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
        self.__max_motion_parameters = rospy.Publisher(
            '/fetch/max_motion_parameters',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/calc_com_total/com_fin',  #change
            Float64MultiArray,
            self.__center_of_mass_robot_callback,
        )

        rospy.Subscriber(
            '/fetch/current_motion_parameters',
            Float64MultiArray,
            self.__current_motion_parameters_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __current_motion_parameters_callback(self, message):
        self.__current_linear_acceleration = message.data[0]

    def __center_of_mass_robot_callback(self, message):
        self.__robot_total_mass = [message.data[0], message.data[1], message.data[2]]
        self.__total_mass_robot = message.data[3]

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
    def max_acc_calc(self, current_linear_acc):  #deal with target acc
        center_of_mass_distance_to_origin = math.sqrt(
            self.__robot_total_mass[0] * self.__robot_total_mass[0]
            + self.__robot_total_mass[1] * self.__robot_total_mass[1]
        )
        e = 2.7
        x_limit_coordinate = -0.221 / e
        y_limit_coordinate = -0.221 / e
        g = 9.81
        acc_centrifugal_max = 0
        acc_tangential_max = 0

        cosa = self.__robot_total_mass[0] / center_of_mass_distance_to_origin
        sina = self.__robot_total_mass[1] / center_of_mass_distance_to_origin
        tga = self.__robot_total_mass[1] / self.__robot_total_mass[0]

        acc_x_axis_max = (
            x_limit_coordinate * self.__total_mass_robot * g - self.__total_mass_robot * self.__robot_total_mass[0]
        ) / (-self.__total_mass_robot * self.__robot_total_mass[2])

        acc_y_axis_max = (
            y_limit_coordinate * self.__total_mass_robot * g - self.__total_mass_robot * self.__robot_total_mass[1]
        ) / (-self.__total_mass_robot * self.__robot_total_mass[2])

        acc_centrifugal_max = (acc_x_axis_max - tga * acc_y_axis_max
                   - current_linear_acc) / (-cosa - tga * sina)

        acc_tangential_max = (acc_y_axis_max - sina * acc_centrifugal_max) / (cosa)

        acc_centrifugal_max = abs(acc_centrifugal_max)
        acc_tangential_max = abs(acc_tangential_max)

        max_rotation_velocity = math.sqrt(acc_centrifugal_max / center_of_mass_distance_to_origin)
        max_rototation_acceleration = acc_tangential_max / center_of_mass_distance_to_origin

        max_linear_acceleration = 0.8 * acc_x_axis_max

        float64_array = Float64MultiArray()
        float64_array.data = [max_linear_acceleration, max_rototation_acceleration, max_rotation_velocity]

        self.__max_motion_parameters.publish(float64_array)

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.max_acc_calc(self.__current_linear_acceleration)

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
    rospy.init_node('zmp_forw_acc')

    class_instance = zmp_forw()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()