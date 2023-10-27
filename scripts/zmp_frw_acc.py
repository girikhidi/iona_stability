#!/usr/bin/env python
"""

"""
import rospy

from std_msgs.msg import (Float64MultiArray, Bool)
import math
import numpy as np

class ZMP_Forw():
    """
    
    """

    def __init__(
        self 
    ):
        """
        
        """

        # # Private constants:
        self.NODE_NAME='zmp_forw_acc'
        self.acc_x=0
        self.acc_y=0
        self.Mass_pos_fin=[0,0,0]
        self.Mass_rob=0
        self.MAX_LINEAR_SPEED=0.5

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
        #Change 
        self.__publisher = rospy.Publisher(
            f'{self.NODE_NAME}/zmp_acc_max',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:

        rospy.Subscriber(
             'calc_com_total/com_fin', #change
            Float64MultiArray,
            self.__COM_Total_callback,
        )
        




    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name> is_initialized topic.
        
        """

        self.__dependency_status['dependency_node_name'] = message.data



    # # Topic callbacks:

    def __COM_Total_callback(self, message):
        self.Mass_pos_fin=[message.data[0], message.data[1], message.data[2]]
        self.Mass_rob=message.data[3]

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
    def max_acc_calc(self, target_linear_acc): #deal with target acc
        R=math.sqrt(self.Mass_pos_fin[0]*self.Mass_pos_fin[0]+self.Mass_pos_fin[1]*self.Mass_pos_fin[1])
        e=2.7
        XlimLow=-0.221/e
        YlimLow=-0.221/e
        g=9.81

        cosa=self.Mass_pos_fin[0]/R
        sina=self.Mass_pos_fin[1]/R
        tga=self.Mass_pos_fin[1]/self.Mass_pos_fin[0]
        

        acc_X_max=(XlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[0])/(-self.Mass_rob*self.Mass_pos_fin[2])
       
        acc_Y_max=(YlimLow*self.Mass_rob*g-self.Mass_rob*self.Mass_pos_fin[1])/(-self.Mass_rob*self.Mass_pos_fin[2])
      

        a_c_max=(acc_X_max-tga*acc_Y_max-target_linear_acc)/(-cosa-tga*sina)
        a_t_max=(acc_Y_max-sina*a_c_max)/(cosa)
        max_rot_vel=math.sqrt(a_c_max/R)
        max_rot_acc=a_t_max/R

        max_linea_acc=0.8*acc_X_max

        float64_array = Float64MultiArray()
        float64_array.data=[max_linea_acc, max_rot_acc, max_rot_vel]

        self.__publisher.publish(float64_array)
    

    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        self.max_acc_calc(target_linear_acc)
        
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

    class_instance = ZMP_Forw()

    rospy.on_shutdown(class_instance.node_shutdown)

    while not rospy.is_shutdown():
        class_instance.main_loop()


if __name__ == '__main__':
    main()