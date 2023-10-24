#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from sensor_msgs.msg import JointState
from math import cos
from math import sin
import numpy as np


class CoF_arm_calc():
    """
    
    """

    def __init__(self):
        """
        
        """

        # # Private constants:

        self.NODE_NAME = 'calc_com_arm_r'
        self.Mass_pos_robot_fin = [0.0, 0.0, 0.0, 0.0]
        self.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
            self.__COF_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data

    # # Topic callbacks:
    def __COF_callback(self, message):
        self.q = message.position

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
        mb = 1.697
        m1 = 1.377
        m2 = 1.1636
        m3 = 1.1636
        m4 = 0.930
        m5 = 0.678
        m6 = 0.678
        mn = 0.500
        ml = 0
        mgrip = 0.925

        q = self.q

        T01 = np.array(
            [
                [cos(q[0]), -sin(q[0]), 0, 0], [-sin(q[0]), -cos(q[0]), 0, 0],
                [0, 0, -1, 0.1564], [0, 0, 0, 1]
            ]
        )
        T12 = np.array(
            [
                [cos(q[1]), -sin(q[1]), 0, 0], [0, 0, -1, 0.0054],
                [sin(q[1]), cos(q[1]), 0, -0.1284], [0, 0, 0, 1]
            ]
        )
        T23 = np.array(
            [
                [cos(q[2]), -sin(q[2]), 0, 0], [0, 0, 1, -0.2104],
                [-sin(q[2]), -cos(q[2]), 0, -0.0064], [0, 0, 0, 1]
            ]
        )
        T34 = np.array(
            [
                [cos(q[3]), -sin(q[3]), 0, 0], [0, 0, -1, -0.0064],
                [sin(q[3]), cos(q[3]), 0, -0.2104], [0, 0, 0, 1]
            ]
        )
        T45 = np.array(
            [
                [cos(q[4]), -sin(q[4]), 0, 0], [0, 0, 1, -0.2084],
                [-sin(q[4]), -cos(q[4]), 0, -0.0064], [0, 0, 0, 1]
            ]
        )
        T56 = np.array(
            [
                [cos(q[5]), -sin(q[5]), 0, 0], [0, 0, -1, 0],
                [sin(q[5]), cos(q[5]), 0, -0.1059], [0, 0, 0, 1]
            ]
        )
        T67 = np.array(
            [
                [cos(q[6]), -sin(q[6]), 0, 0], [0, 0, 1, -0.1059],
                [-sin(q[6]), -cos(q[6]), 0, 0], [0, 0, 0, 1]
            ]
        )
        T7L = np.array(
            [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, -0.0615], [0, 0, 0, 1]]
        )

        gripper_com = np.array([0, 0, 0.058])
        load_com = np.array([0, 0, 0.08765])

        Mass_posb = np.array([-0.000648, -0.000166, 0.084487])
        Mass_pos1_1 = np.array([-0.000023, -0.010364, -0.073360, 1])
        Mass_pos1 = np.dot(T01, Mass_pos1_1.transpose())
        Mass_pos2_1 = np.array([-0.000044, -0.099580, -0.013278, 1])
        Mass_pos2 = np.dot(np.dot(T01, T12), Mass_pos2_1.transpose())
        Mass_pos3_1 = np.array([-0.000044, -0.006641, -0.117892, 1])
        Mass_pos3 = np.dot(
            np.dot(np.dot(T01, T12), T23), Mass_pos3_1.transpose()
        )
        Mass_pos4_1 = np.array([-0.000018, -0.075478, -0.015006, 1])
        Mass_pos4 = np.dot(
            np.dot(np.dot(np.dot(T01, T12), T23), T34), Mass_pos4_1.transpose()
        )
        Mass_pos5_1 = np.array([0.000001, -0.009432, -0.063883, 1])
        Mass_pos5 = np.dot(
            np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45),
            Mass_pos5_1.transpose()
        )
        Mass_pos6_1 = np.array([0.000001, -0.045483, -0.009650, 1])
        Mass_pos6 = np.dot(
            np.dot(
                np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56
            ), Mass_pos6_1.transpose()
        )
        Mass_posN_1 = np.array([-0.000281, -0.011402, -0.029798, 1])
        Mass_posN = np.dot(
            np.dot(
                np.dot(
                    np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56
                ), T67
            ), Mass_posN_1.transpose()
        )

        Pos_L = np.dot(
            np.dot(
                np.dot(
                    np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56
                ), T67
            ), T7L
        )

        ml_f = ml + mgrip
        Mass_posL = np.array(
            [0, 0, (gripper_com[2] * mgrip + load_com[2] * ml) / ml_f, 1]
        )
        Mass_posL = np.dot(Pos_L, Mass_posL.transpose())

        X = (
            Mass_posb[0] * mb + Mass_pos1[0] * m1 + Mass_pos2[0] * m2
            + Mass_pos3[0] * m3 + Mass_pos4[0] * m4 + Mass_pos5[0] * m5
            + Mass_pos6[0] * m6 + Mass_posN[0] * mn + Mass_posL[0] * ml_f
        ) / (mb + m1 + m2 + m3 + m4 + m5 + m6 + mn + ml_f)

        Y = (
            Mass_posb[1] * mb + Mass_pos1[1] * m1 + Mass_pos2[1] * m2
            + Mass_pos3[1] * m3 + Mass_pos4[1] * m4 + Mass_pos5[1] * m5
            + Mass_pos6[1] * m6 + Mass_posN[1] * mn + Mass_posL[1] * ml_f
        ) / (mb + m1 + m2 + m3 + m4 + m5 + m6 + mn + ml_f)

        Z = (
            Mass_posb[2] * mb + Mass_pos1[2] * m1 + Mass_pos2[2] * m2
            + Mass_pos3[2] * m3 + Mass_pos4[2] * m4 + Mass_pos5[2] * m5
            + Mass_pos6[2] * m6 + Mass_posN[2] * mn + Mass_posL[2] * ml_f
        ) / (mb + m1 + m2 + m3 + m4 + m5 + m6 + mn + ml_f)

        M_total = mb + m1 + m2 + m3 + m4 + m5 + m6 + mn + ml_f
        self.Mass_pos_robot_fin = [X, Y, Z, M_total]

        float64_array = Float64MultiArray()
        float64_array.data = self.Mass_pos_robot_fin
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

    class_instance = CoF_arm_calc()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()
