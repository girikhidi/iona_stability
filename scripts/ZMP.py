#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
from math import sqrt
from  sensor_msgs.msg import (Imu)

class zmp_simple():
    """
    
    """

    def __init__(
        self 
    ):
        """
        
        """

        # # Private constants:
        self.NODE_NAME='zmp_simple'
        self.__imu_acc_x_axis=0
        self.__imu_acc_y_axis=0
        self.__center_of_mass_robot=[0.001,0.001,0.001]
        self.__mass_robot=131
        self.__flag=0
        self.__count=0

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
    

        # # Topic subscriber:
        rospy.Subscriber(
            '/fetch/imu1', #change
            Imu,
            self.__imu_base_callback,
        )

        rospy.Subscriber(
            'calc_com_total/com_fin', #change
            Float64MultiArray,
            self.__center_of_mass_robot_callback,
        )



    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data



    # # Topic callbacks:

    def __imu_base_callback(self, message):
        self.__imu_acc_x_axis = message.linear_acceleration[0]
        self.__imu_acc_y_axis = message.linear_acceleration[1]
    
    def __center_of_mass_robot_callback(self, message):
        self.__center_of_mass_robot=[message.data[0], message.data[1], message.data[2]]
        self.__mass_robot=message.data[3]
        
 

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

    def __zmp_ass(self):
        
        g=9.81
        e = 2.7

        acc_robot=[self.__imu_acc_x_axis, self.__imu_acc_y_axis, 0]
        x_coordinate_zmp=(self.__mass_robot*(acc_robot[2]+g)*self.__center_of_mass_robot[0]-self.__mass_robot*self.__center_of_mass_robot[2]*acc_robot[0])/(self.__mass_robot*(acc_robot[2]+g))
        y_coordinate_zmp=(self.__mass_robot*(acc_robot[2]+g)*self.__center_of_mass_robot[1]-self.__mass_robot*self.__center_of_mass_robot[2]*acc_robot[1])/(self.__mass_robot*(acc_robot[2]+g))

        x_limit_coordinate=-0.221/e
        y_limit_coordinate=-0.221/e

        if abs(x_coordinate_zmp)>abs(x_limit_coordinate) and abs(y_coordinate_zmp)>abs(y_limit_coordinate):
            self.__flag=self.__flag+1
        elif abs(x_coordinate_zmp)<=abs(x_limit_coordinate) and abs(y_coordinate_zmp)>abs(y_limit_coordinate):
            self.__flag=self.__flag+1
        elif abs(x_coordinate_zmp)>abs(x_limit_coordinate) and abs(y_coordinate_zmp)<=abs(y_limit_coordinate):
            self.__flag=self.__flag+1
        elif abs(x_coordinate_zmp)<=abs(x_limit_coordinate) and abs(y_coordinate_zmp)<=abs(y_limit_coordinate):
            self.__flag=0
        
        if self.__flag==1:
            self.__count=self.__count+1
            print(f'warning {self.__count}')
        

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__zmp_ass()
        
        
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
    rospy.init_node('zmp_simple')

    class_instance = zmp_simple()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()