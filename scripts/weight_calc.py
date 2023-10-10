#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from sensor_msgs.msg import JointState
from math import cos
from math import sin
import numpy as np


class Load_Calc():
    """
    
    """

    def __init__(
        self 
    ):
        """
        
        """

        # # Private constants:
        self.NODE_NAME='load_calc'
        self.q=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.t=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
            'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'dependency_node_name':
                rospy.Subscriber(
                    f'/dependency_node_name/is_initialized',
                    Bool,
                    self.__dependency_name_callback,
                ),
        }

        # # Topic publisher:
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/weight_load',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            'MyRobotL/joint_state', #change
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
        self.t = message.effort
        self.q=self.q[0:6]
        self.t=self.t[0:6]

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
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        q=self.q
        t=self.t
        
        T01=np.array([[cos(q[0]), -sin(q[0]), 0, 0],
        [-sin(q[0]), -cos(q[0]), 0, 0],
        [0, 0, -1, 0.1564],
        [0, 0, 0, 1]])
        T12=np.array([[cos(q[1]), -sin(q[1]), 0, 0],
            [0, 0, -1, 0.0054],
            [sin(q[1]), cos(q[1]), 0, -0.1284],
            [0, 0, 0, 1]])
        T23=np.array([[cos(q[2]), -sin(q[2]), 0, 0],
            [0, 0, 1, -0.2104],
            [-sin(q[2]), -cos(q[2]), 0, -0.0064],
            [0, 0, 0, 1]])
        T34=np.array([[cos(q[3]), -sin(q[3]), 0, 0],
            [0, 0, -1, -0.0064],
            [sin(q[3]), cos(q[3]), 0, -0.2104],
            [0, 0, 0, 1]])
        T45=np.array([[cos(q[4]), -sin(q[4]), 0, 0],
            [0, 0, 1, -0.2084],
            [-sin(q[4]), -cos(q[4]), 0, -0.0064],
            [0, 0, 0, 1]])
        T56=np.array([[cos(q[5]), -sin(q[5]), 0, 0],
            [0, 0, -1, 0],
            [sin(q[5]), cos(q[5]), 0, -0.1059],
            [0, 0, 0, 1]])
        T6L=np.array([[1, 0, 0, 0],
            [0, 0, -1, -0.3069],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])
        
        T1=np.dot(T01,T12)
        T2=np.dot(np.dot(T01,T12),T23)
        T3=np.dot(np.dot(np.dot(T01,T12),T23),T34)
        T4=np.dot(np.dot(np.dot(np.dot(T01,T12),T23),T34),T45)
        T5=np.dot(np.dot(np.dot(np.dot(np.dot(T01,T12),T23),T34),T45),T56)
        T7=np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T01,T12),T23),T34),T45),T56),T6L)
        o7=T7[0:3, 3]
        o0=T01[0:3, 3]
        z0=T01[0:3, 2]
        J1=np.append(np.cross(z0, o7-o0), z0).reshape(-1, 1)
        
        o1=T1[0:3, 3]
        z1=T1[0:3, 2]
        J2=np.append(np.cross(z1, o7-o1), z1).reshape(-1, 1)

        o2=T2[0:3, 3]
        z2=T2[0:3, 2]
        J3=np.append(np.cross(z2, o7-o2), z2).reshape(-1, 1)

        o3=T3[0:3, 3]
        z3=T3[0:3, 2]
        J4=np.append(np.cross(z3, o7-o3), z3).reshape(-1, 1)

        o4=T4[0:3, 3]
        z4=T4[0:3, 2]
        J5=np.append(np.cross(z4, o7-o4), z4).reshape(-1, 1)

        o5=T5[0:3, 3]
        z5=T5[0:3, 2]
        J6=np.append(np.cross(z5, o7-o5), z5).reshape(-1, 1)
        J=np.hstack((np.hstack((np.hstack((np.hstack((np.hstack((J1, J2)), J3)), J4)), J5)), J6))
        rank = np.linalg.matrix_rank(J)

        if rank==6:
            JT=J.transpose()
            JInv=np.linalg.inv(JT)
            t_np=np.array(t)
            weight_np=np.dot(t_np, JInv).tolist()
        else:
            rospy.loginfo_once(f'{self.NODE_NAME}: Rank<6')

        
        float64_array = Float64MultiArray()
        float64_array.data=weight_np
        
        self.__publisher.publish(float64_array)
        

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
    rospy.init_node('load_calc')

    class_instance = Load_Calc()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()
